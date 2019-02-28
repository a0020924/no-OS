/***************************************************************************//**
 *   @file   network.c
 *   @brief  Implementation of network interface.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <errno.h>
#include "lwip/tcp.h"
#include "uthash.h"
#include "lwip_init.h"
#include "network.h"
#include "comm_util.h"

#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
#endif

enum network_states {
	ES_NONE = 0,
	ES_ACCEPTED,
	ES_RECEIVED,
	ES_CLOSING
};

struct network_instance {
	int32_t instance_id;
	u8_t state;
	u8_t retries;
	struct tcp_pcb *pcb;
	/* pbuf (chain) to recycle */
	struct pbuf *p;
	UT_hash_handle hh;         /* makes this structure hashable */
};

static struct fifo *network_fifo = NULL;
static struct network_instance *instances = NULL;

err_t network_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t network_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void network_error(void *arg, err_t err);
void network_store(struct tcp_pcb *tpcb, struct network_instance *es);
void network_close(struct tcp_pcb *tpcb, struct network_instance *es);
err_t network_poll(void *arg, struct tcp_pcb *tpcb);

/***************************************************************************//**
 * @brief network_keep_alive
*******************************************************************************/
void network_keep_alive(void)
{
	lwip_keep_alive();
}

/***************************************************************************//**
 * @brief network_init
*******************************************************************************/
int32_t network_init(void)
{
	set_keep_alive(lwip_keep_alive);
	return init_lwip();
}

/***************************************************************************//**
 * @brief start_application
*******************************************************************************/
void network_start(void)
{
	struct tcp_pcb *pcb;
	u16_t port = 30431;
	pcb = tcp_new();
	if (pcb != NULL) {
		err_t err;

		err = tcp_bind(pcb, IP_ADDR_ANY, port);
		if (err == ERR_OK) {
			pcb = tcp_listen(pcb);
			tcp_accept(pcb, network_accept);
			xil_printf("tinyiiod server started @ port %d\n\r", port);
		} else {
			/* abort? output diagnostic? */
		}
	} else {
		/* abort? output diagnostic? */
	}
}

/***************************************************************************//**
 * @brief network_accept
*******************************************************************************/
err_t network_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	err_t ret_err;
	struct network_instance *es;
	static int32_t inst_id = 0;
	inst_id++;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	tcp_setprio(newpcb, TCP_PRIO_MAX);

	es = (struct network_instance *)mem_malloc(sizeof(struct network_instance));
	if (es != NULL) {
		es->state = ES_ACCEPTED;
		es->pcb = newpcb;
		es->retries = 0;
		es->p = NULL;
		es->instance_id = inst_id;
		/* cppcheck-suppress uninitvar ; cppcheck reports this as a false positive */
		HASH_ADD_INT( instances, instance_id, es );
#ifdef DEBUG_NETWORK
		printf("new clientconnected: %d\n", inst_id);
#endif
		/* pass newly allocated es to our callbacks */
		tcp_arg(newpcb, es);
		tcp_recv(newpcb, network_recv);
		tcp_err(newpcb, network_error);
		ret_err = ERR_OK;
	} else {
		ret_err = ERR_MEM;
	}

	return ret_err;
}

/***************************************************************************//**
 * @brief network_recv
*******************************************************************************/
err_t network_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	struct network_instance *es;
	err_t ret_err;
	LWIP_ASSERT("arg != NULL",arg != NULL);
	es = (struct network_instance *)arg;
	if (p == NULL) {
		/* remote host closed connection */
		es->state = ES_CLOSING;
		if(es->p == NULL) {
			/* we're done sending, close it */
			network_close(tpcb, es);
		} else {
			/* we're not done yet */
			network_store(tpcb, es);
		}
		ret_err = ERR_OK;
	} else if(err != ERR_OK) {
		/* cleanup, for unkown reason */
		if (p != NULL) {
			es->p = NULL;
			pbuf_free(p);
		}
		ret_err = err;
	} else if(es->state == ES_ACCEPTED) {
		/* first data chunk in p->payload */
		es->state = ES_RECEIVED;
		/* store reference to incoming pbuf (chain) */
		es->p = p;
		/* install send completion notifier */
		network_store(tpcb, es);
		ret_err = ERR_OK;
	} else if (es->state == ES_RECEIVED) {
		/* read some more data */
		if(es->p == NULL) {
			es->p = p;
			network_store(tpcb, es);
		} else {
			struct pbuf *ptr;

			/* chain pbufs to the end of what we recv'ed previously  */
			ptr = es->p;
			pbuf_chain(ptr,p);
		}
		ret_err = ERR_OK;
	} else if(es->state == ES_CLOSING) {
		/* odd case, remote side closing twice, trash data */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		network_close(tpcb, es);
		ret_err = ERR_OK;
	} else {
		/* unkown es->state, trash data  */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		ret_err = ERR_OK;
	}

	return ret_err;
}

/***************************************************************************//**
 * @brief network_error
*******************************************************************************/
void network_error(void *arg, err_t err)
{
	struct network_instance *es;

	LWIP_UNUSED_ARG(err);

	es = (struct network_instance *)arg;
	if (es != NULL) {
		mem_free(es);
	}
}

/***************************************************************************//**
 * @brief network_store
*******************************************************************************/
void network_store(struct tcp_pcb *tpcb, struct network_instance *es)
{
	struct pbuf *ptr;
	while (es->p != NULL) {
		u16_t plen;
		u8_t freed;
		ptr = es->p;
		plen = ptr->len;

		fifo_insert_tail(&network_fifo, ptr->payload, ptr->len, es->instance_id);
		/* continue with next pbuf in chain (if any) */
		es->p = ptr->next;
		if(es->p != NULL) {
			/* new reference! */
			pbuf_ref(es->p);
		}
		/* chop first pbuf from chain */
		do {
			/* try hard to free pbuf */
			freed = pbuf_free(ptr);
		} while(freed == 0);
		/* we can read more data now */
		tcp_recved(tpcb, plen);
	}
}

/***************************************************************************//**
 * @brief network_close
*******************************************************************************/
void network_close(struct tcp_pcb *tpcb, struct network_instance *es)
{
	while(network_fifo) {
		network_fifo = fifo_remove_head(network_fifo);
	}
	tcp_arg(tpcb, NULL);
	tcp_sent(tpcb, NULL);
	tcp_recv(tpcb, NULL);
	tcp_err(tpcb, NULL);
	tcp_poll(tpcb, NULL, 0);

	if (es != NULL) {
		mem_free(es);
	}
	tcp_close(tpcb);
}

/***************************************************************************//**
 * @brief network_read_line
*******************************************************************************/
int32_t network_read_line(int32_t *instance_id, char *buf, size_t len)
{
	return comm_read_line(&network_fifo, instance_id, buf, len);
}

/***************************************************************************//**
 * @brief network_read
*******************************************************************************/
int32_t network_read(int32_t *instance_id, char *buf, size_t len)
{
	return comm_read(&network_fifo, instance_id, buf, len);
}

/***************************************************************************//**
 * @brief network_write_data
*******************************************************************************/
void network_write_data(int32_t instance_id, const char *buf, size_t len)
{
	struct network_instance *instance = NULL;
	u8_t apiflags = TCP_WRITE_FLAG_COPY;
	const char *pbuffer = buf;
	/* cppcheck-suppress uninitvar ; cppcheck reports this as a false positive */
	HASH_FIND_INT( instances, &instance_id, instance);
	if(instance == NULL)
		return;
	do {
		do {
			lwip_keep_alive();
		} while(tcp_sndbuf(instance->pcb) == 0);

		int32_t buf_len = tcp_sndbuf(instance->pcb);
		int32_t wr_length = buf_len > len ? len : buf_len;
		apiflags |= buf_len > len ? 0 : TCP_WRITE_FLAG_MORE;
		tcp_write(instance->pcb, pbuffer, wr_length, apiflags);
		tcp_output(instance->pcb);
		pbuffer += wr_length;
		len -= wr_length;
	} while(len);
	do {
		lwip_keep_alive();
	} while(tcp_sndbuf(instance->pcb) == 0);
}

/***************************************************************************//**
 * @brief network_exit
*******************************************************************************/
int32_t network_close_instance(int32_t instance_id)
{
	struct network_instance *instance;
	err_t err = 0;
#ifdef DEBUG_NETWORK
	printf("removing cleint instance: %d\n", instance_id);
#endif
	/* cppcheck-suppress uninitvar ; cppcheck reports this as a false positive */
	HASH_FIND_INT( instances, &instance_id, instance);
	if(instance == NULL)
		return -ENOENT;
	HASH_DEL(instances, instance);
	network_close(instance->pcb, instance);
#ifdef DEBUG_NETWORK
	printf("removed client inst %d done\n", instance_id);
#endif

	return err;
}
