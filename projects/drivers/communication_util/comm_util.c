/***************************************************************************//**
 *   @file   comm_util.c
 *   @brief  Implementation of fifo.
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
#include <comm_util.h>
#include <string.h>

static void (*keep_alive)(void) = NULL;

/***************************************************************************//**
 * @brief new_buffer
*******************************************************************************/
static struct fifo * new_buffer()
{
	struct fifo *buf = malloc(sizeof(struct fifo));
	buf->len = 0;
	buf->data = NULL;
	buf->next = NULL;

	return buf;
}

/***************************************************************************//**
 * @brief get_last
*******************************************************************************/
static struct fifo *get_last(struct fifo *p_fifo)
{
	if(p_fifo == NULL)
		return NULL;
	while (p_fifo->next) {
		p_fifo = p_fifo->next;
	}

	return p_fifo;
}

/***************************************************************************//**
 * @brief fifo_insert_tail
*******************************************************************************/
void fifo_insert_tail(struct fifo **p_fifo, char *buff, int32_t len,  int32_t id)
{
	struct fifo *p = NULL;
	if(*p_fifo == NULL) {
		p = new_buffer();
		*p_fifo = p;
	} else {
		p = get_last(*p_fifo);
		p->next = new_buffer();
		p = p->next;
	}
	p->instance_id = id;
	p->data = malloc(len);
	memcpy(p->data, buff, len);
	p->len = len;
}

/***************************************************************************//**
 * @brief fifo_remove_head
*******************************************************************************/
struct fifo * fifo_remove_head(struct fifo *p_fifo)
{
	struct fifo *p = p_fifo;
	if(p_fifo != NULL) {
		p_fifo = p_fifo->next;
		free(p->data);
		p->len = 0;
		p->next = NULL;
		p->data = NULL;
		free(p);
		p = NULL;
	}

	return p_fifo;
}

/***************************************************************************//**
 * @brief set_keep_alive
*******************************************************************************/
void set_keep_alive(void (*kp_alive)(void)) {
	keep_alive = kp_alive;
}

/***************************************************************************//**
 * @brief comm_read_line
*******************************************************************************/
int32_t comm_read_line(struct fifo **fifo, int32_t *instance_id, char *buf, size_t len)
{
	int32_t length = 0;
	char *data = NULL;
	while(*fifo == NULL) {
		if(keep_alive)
			keep_alive();
	}

	data = (*fifo)->data;
	char* end = strstr(data, "\r\n");
	if(end && end == data) { /* \r\n on first pos */
		(*fifo)->len -= 2;
		data += 2;
		end = strstr(data, "\r\n");
	}
	*instance_id = (*fifo)->instance_id;
	if(end) {
		length = end - data;
		memcpy(buf, data, length);
		buf[length] = '\0';
		if(length + 2 >= (*fifo)->len) {
			*fifo = fifo_remove_head(*fifo);
		} else {
			(*fifo)->len = (*fifo)->len - length - 2;
			char * remaining = malloc((*fifo)->len);
			memcpy(remaining, (end + 2), (*fifo)->len);
			free((*fifo)->data);
			(*fifo)->data = remaining;
		}
	} else {
		memcpy(buf, (*fifo)->data, (*fifo)->len);
		buf[length] = '\0';
		*fifo = fifo_remove_head(*fifo);
	}

	return length;
}

/***************************************************************************//**
 * @brief comm_read
*******************************************************************************/
int32_t comm_read(struct fifo **network_fifo, int32_t *instance_id, char *buf, size_t len)
{
	int32_t temp_len = 0;
	while(*network_fifo == NULL) {
		if(keep_alive)
			keep_alive();
	}
	*instance_id = (*network_fifo)->instance_id;
	if((*network_fifo)->len == len) {
		memcpy(buf, (*network_fifo)->data, len);
		(*network_fifo) = fifo_remove_head(*network_fifo);
		temp_len =  len;
	} else if ((*network_fifo)->len < len) {
		char *pbuf = buf;
		do {
			if(*network_fifo) {
				memcpy(pbuf, (*network_fifo)->data, (*network_fifo)->len);
				pbuf = pbuf + (*network_fifo)->len;
				temp_len += (*network_fifo)->len;
				*network_fifo = fifo_remove_head(*network_fifo);
			}
			if(keep_alive && temp_len < len)
				keep_alive();
		} while(temp_len < len);
	} else {
		memcpy(buf, (*network_fifo)->data, len);
		(*network_fifo)->len = (*network_fifo)->len - len; /* new length */
		char * remaining = malloc((*network_fifo)->len);
		memcpy(remaining, (*network_fifo)->data + len, (*network_fifo)->len);
		free((*network_fifo)->data);
		(*network_fifo)->data = remaining;
		temp_len =  len;
	}

	return temp_len;
}
