/***************************************************************************//**
 *   @file   Platform.c
 *   @brief  Implementation of Platform Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <xparameters.h>
#ifdef _XPARAMETERS_PS_H_
#include <xgpiops.h>
#include <xspips.h>
#else
#include <xgpio.h>
#include <xgpio_l.h>
#include <xspi.h>
#endif
#include "util.h"
#include "adc_core.h"
#include "dac_core.h"
#include "platform.h"
#ifdef _XPARAMETERS_PS_H_
#include <sleep.h>
#else
static inline void usleep(unsigned long usleep)
{
	unsigned long delay = 0;

	for(delay = 0; delay < usleep * 10; delay++);
}
#endif

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
#ifdef _XPARAMETERS_PS_H_
XSpiPs_Config	*spi_config;
XSpiPs			spi_instance;
XGpioPs_Config	*gpio_config;
XGpioPs			gpio_instance;
#else
XSpi_Config		*spi_config;
XSpi			spi_instance;
XGpio_Config	*gpio_config;
#endif

/**
 * @brief Initialize the SPI communication peripheral.
 * @param desc - The SPI descriptor.
 * @param init_param - The structure that contains the SPI parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_init(struct spi_desc **desc,
		 const struct spi_init_param *param)
{
	spi_desc *descriptor;
	int32_t ret;

	descriptor = (struct spi_desc *) calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return FAILURE;

	descriptor->mode = param->mode;
	descriptor->chip_select = param->chip_select;
	descriptor->flags = param->flags;

#ifdef _XPARAMETERS_PS_H_
	descriptor->config = XSpiPs_LookupConfig(param->id);
	if (descriptor->config == NULL)
		goto error;

	ret = XSpiPs_CfgInitialize(&descriptor->instance,
				   descriptor->config, descriptor->config->BaseAddress);
	if (ret != 0)
		goto error;

	XSpiPs_SetOptions(&descriptor->instance,
			  XSPIPS_MASTER_OPTION |
			  ((descriptor->flags & SPI_CS_DECODE) ?
			   XSPIPS_DECODE_SSELECT_OPTION : 0) |
			  XSPIPS_FORCE_SSELECT_OPTION |
			  ((descriptor->mode & SPI_CPOL) ?
			   XSPIPS_CLK_ACTIVE_LOW_OPTION : 0) |
			  ((descriptor->mode & SPI_CPHA) ?
			   XSPIPS_CLK_PHASE_1_OPTION : 0));

	XSpiPs_SetClkPrescaler(&descriptor->instance,
			       XSPIPS_CLK_PRESCALE_64);

	XSpiPs_SetSlaveSelect(&descriptor->instance, 0xf);
#else
	ret = XSpi_Initialize(&descriptor->instance, param->id);
	if (ret != 0)
		goto error;

	XSpi_SetOptions(&descriptor->instance,
			XSP_MASTER_OPTION |
			((descriptor->mode & SPI_CPOL) ?
			 XSP_CLK_ACTIVE_LOW_OPTION : 0) |
			((descriptor->mode & SPI_CPHA) ?
			 XSP_CLK_PHASE_1_OPTION : 0));

	XSpi_Start(&descriptor->instance);

	XSpi_IntrGlobalDisable(&descriptor->instance);
#endif

	*desc = descriptor;

	return SUCCESS;

error:
	free(descriptor);

	return FAILURE;
}

/**
 * @brief Write and read data to/from SPI.
 * @param desc - The SPI descriptor.
 * @param data - The buffer with the transmitted/received data.
 * @param bytes_number - Number of bytes to write/read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_write_and_read(struct spi_desc *desc,
			   uint8_t *data,
			   uint8_t bytes_number)
{
#ifdef _XPARAMETERS_PS_H_
	XSpiPs_SetOptions(&desc->instance,
			  XSPIPS_MASTER_OPTION |
			  ((desc->flags & SPI_CS_DECODE) ?
			   XSPIPS_DECODE_SSELECT_OPTION : 0) |
			  XSPIPS_FORCE_SSELECT_OPTION |
			  ((desc->mode & SPI_CPOL) ?
			   XSPIPS_CLK_ACTIVE_LOW_OPTION : 0) |
			  ((desc->mode & SPI_CPHA) ?
			   XSPIPS_CLK_PHASE_1_OPTION : 0));

	XSpiPs_SetSlaveSelect(&desc->instance,
			      0xf & ~desc->chip_select);
	XSpiPs_PolledTransfer(&desc->instance,
			      data, data, bytes_number);
#else
	XSpi_SetOptions(&desc->instance,
			XSP_MASTER_OPTION |
			((desc->mode & SPI_CPOL) ?
			 XSP_CLK_ACTIVE_LOW_OPTION : 0) |
			((desc->mode & SPI_CPHA) ?
			 XSP_CLK_PHASE_1_OPTION : 0));

	XSpi_SetSlaveSelect(&desc->instance,
			    desc->chip_select);

	XSpi_Transfer(&desc->instance,
		      data, data, bytes_number);
#endif
	return 0;
}

/***************************************************************************//**
 * @brief gpio_init
*******************************************************************************/
void gpio_init(uint32_t device_id)
{
#ifdef _XPARAMETERS_PS_H_
	gpio_config = XGpioPs_LookupConfig(device_id);
	XGpioPs_CfgInitialize(&gpio_instance, gpio_config, gpio_config->BaseAddr);
#else
	gpio_config = XGpio_LookupConfig(device_id);
#endif
}

/***************************************************************************//**
 * @brief gpio_direction
*******************************************************************************/
void gpio_direction(uint8_t pin, uint8_t direction)
{
#ifdef _XPARAMETERS_PS_H_
	XGpioPs_SetDirectionPin(&gpio_instance, pin, direction);
	XGpioPs_SetOutputEnablePin(&gpio_instance, pin, 1);
#else
	uint32_t config = 0;
	uint32_t tri_reg_addr;

	if (pin >= 32) {
		tri_reg_addr = XGPIO_TRI2_OFFSET;
		pin -= 32;
	} else
		tri_reg_addr = XGPIO_TRI_OFFSET;

	config = Xil_In32((gpio_config->BaseAddress + tri_reg_addr));
	if(direction) {
		config &= ~(1 << pin);
	} else {
		config |= (1 << pin);
	}
	Xil_Out32((gpio_config->BaseAddress + tri_reg_addr), config);
#endif
}

/***************************************************************************//**
 * @brief gpio_is_valid
*******************************************************************************/
bool gpio_is_valid(int number)
{
	if(number >= 0)
		return 1;
	else
		return 0;
}

/***************************************************************************//**
 * @brief gpio_data
*******************************************************************************/
void gpio_data(uint8_t pin, uint8_t data)
{
#ifdef _XPARAMETERS_PS_H_
	XGpioPs_WritePin(&gpio_instance, pin, data);
#else
	uint32_t config = 0;
	uint32_t data_reg_addr;

	if (pin >= 32) {
		data_reg_addr = XGPIO_DATA2_OFFSET;
		pin -= 32;
	} else
		data_reg_addr = XGPIO_DATA_OFFSET;

	config = Xil_In32((gpio_config->BaseAddress + data_reg_addr));
	if(data) {
		config |= (1 << pin);
	} else {
		config &= ~(1 << pin);
	}
	Xil_Out32((gpio_config->BaseAddress + data_reg_addr), config);
#endif
}

/***************************************************************************//**
 * @brief gpio_set_value
*******************************************************************************/
void gpio_set_value(unsigned gpio, int value)
{
	gpio_data(gpio, value);
}

/***************************************************************************//**
 * @brief udelay
*******************************************************************************/
void udelay(unsigned long usecs)
{
	usleep(usecs);
}

/***************************************************************************//**
 * @brief mdelay
*******************************************************************************/
void mdelay(unsigned long msecs)
{
	usleep(msecs * 1000);
}

/***************************************************************************//**
 * @brief msleep_interruptible
*******************************************************************************/
unsigned long msleep_interruptible(unsigned int msecs)
{
	mdelay(msecs);

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_init
*******************************************************************************/
void axiadc_init(struct ad9361_rf_phy *phy)
{
	adc_init(phy);
	dac_init(phy, DATA_SEL_DDS, 0);
}

/***************************************************************************//**
 * @brief axiadc_post_setup
*******************************************************************************/
int axiadc_post_setup(struct ad9361_rf_phy *phy)
{
	return ad9361_post_setup(phy);
}

/***************************************************************************//**
 * @brief axiadc_read
*******************************************************************************/
unsigned int axiadc_read(struct axiadc_state *st, unsigned long reg)
{
	uint32_t val;

	adc_read(st->phy, reg, &val);

	return val;
}

/***************************************************************************//**
 * @brief axiadc_write
*******************************************************************************/
void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	adc_write(st->phy, reg, val);
}

/***************************************************************************//**
 * @brief axiadc_set_pnsel
*******************************************************************************/
int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
	unsigned reg;

	uint32_t version = axiadc_read(st, 0x4000);

	if (PCORE_VERSION_MAJOR(version) > 7) {
		reg = axiadc_read(st, ADI_REG_CHAN_CNTRL_3(channel));
		reg &= ~ADI_ADC_PN_SEL(~0);
		reg |= ADI_ADC_PN_SEL(sel);
		axiadc_write(st, ADI_REG_CHAN_CNTRL_3(channel), reg);
	} else {
		reg = axiadc_read(st, ADI_REG_CHAN_CNTRL(channel));

		if (sel == ADC_PN_CUSTOM) {
			reg |= ADI_PN_SEL;
		} else if (sel == ADC_PN9) {
			reg &= ~ADI_PN23_TYPE;
			reg &= ~ADI_PN_SEL;
		} else {
			reg |= ADI_PN23_TYPE;
			reg &= ~ADI_PN_SEL;
		}

		axiadc_write(st, ADI_REG_CHAN_CNTRL(channel), reg);
	}

	return 0;
}

/***************************************************************************//**
 * @brief axiadc_idelay_set
*******************************************************************************/
void axiadc_idelay_set(struct axiadc_state *st,
		       unsigned lane, unsigned val)
{
	if (PCORE_VERSION_MAJOR(st->pcore_version) > 8) {
		axiadc_write(st, ADI_REG_DELAY(lane), val);
	} else {
		axiadc_write(st, ADI_REG_DELAY_CNTRL, 0);
		axiadc_write(st, ADI_REG_DELAY_CNTRL,
			     ADI_DELAY_ADDRESS(lane)
			     | ADI_DELAY_WDATA(val)
			     | ADI_DELAY_SEL);
	}
}
