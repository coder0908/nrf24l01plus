/*
 * nrf24l01p_driver.c
 *
 *  Created on: Apr 26, 2025
 *      Author: coder0908
 */

#include <assert.h>
#include "nrf24l01p_driver.h"

/*COMMAND*/
#define DRV_NRF24L01P_CMD_R_REGISTER		0b00011111
#define DRV_NRF24L01P_CMD_W_REGISTER		0b00100000
#define DRV_NRF24L01P_CMD_R_RX_PAYLOAD		0b01100001
#define DRV_NRF24L01P_CMD_W_TX_PAYLOAD		0b10100000
#define DRV_NRF24L01P_CMD_FLUSH_TX		0b11100001
#define DRV_NRF24L01P_CMD_FLUSH_RX		0b11100010
#define DRV_NRF24L01P_CMD_REUSE_TX_PL		0b11100011
#define DRV_NRF24L01P_CMD_R_RX_PL_WID		0b01100000
#define DRV_NRF24L01P_CMD_W_ACK_PAYLOAD		0b10101000
#define DRV_NRF24L01P_CMD_W_TX_PAYLOAD_NOACK	0b10110000
#define DRV_NRF24L01P_CMD_NOP			0xff

/*REGISTER*/
#define DRV_NRF24L01P_REG_CONFIG		0x00
#define DRV_NRF24L01P_REG_EN_AA			0x01
#define DRV_NRF24L01P_REG_EN_RXADDR		0x02
#define DRV_NRF24L01P_REG_SETUP_AW		0x03
#define DRV_NRF24L01P_REG_SETUP_RETR		0x04
#define DRV_NRF24L01P_REG_RF_CH			0x05
#define DRV_NRF24L01P_REG_RF_SETUP		0x06
#define DRV_NRF24L01P_REG_STATUS		0x07
#define DRV_NRF24L01P_REG_OBSERVE_TX		0x08
#define DRV_NRF24L01P_REG_RPD			0x09
#define DRV_NRF24L01P_REG_RX_ADDR_P0		0x0A
#define DRV_NRF24L01P_REG_RX_ADDR_P1		0x0B
#define DRV_NRF24L01P_REG_RX_ADDR_P2		0x0C
#define DRV_NRF24L01P_REG_RX_ADDR_P3		0x0D
#define DRV_NRF24L01P_REG_RX_ADDR_P4		0x0E
#define DRV_NRF24L01P_REG_RX_ADDR_P5		0x0F
#define DRV_NRF24L01P_REG_TX_ADDR		0x10
#define DRV_NRF24L01P_REG_RX_PW_P0		0x11
#define DRV_NRF24L01P_REG_RX_PW_P1		0x12
#define DRV_NRF24L01P_REG_RX_PW_P2		0x13
#define DRV_NRF24L01P_REG_RX_PW_P3		0x14
#define DRV_NRF24L01P_REG_RX_PW_P4		0x15
#define DRV_NRF24L01P_REG_RX_PW_P5		0x16
#define DRV_NRF24L01P_REG_FIFO_STATUS		0x17
#define DRV_NRF24L01P_REG_DYNPD			0x1C
#define DRV_NRF24L01P_REG_FEATURE		0x1D

#define _BV(bit) (1 << bit)

static void en_cs(struct drv_nrf24l01p *rd)
{
	assert(rd);
	HAL_GPIO_WritePin(rd->csPort, rd->csPin, GPIO_PIN_RESET);
}

static void dis_cs(struct drv_nrf24l01p *rd)
{
	assert(rd);
	HAL_GPIO_WritePin(rd->csPort, rd->csPin, GPIO_PIN_SET);
}

static void en_ce(struct drv_nrf24l01p *rd)
{
	assert(rd);
	HAL_GPIO_WritePin(rd->cePort, rd->cePin, GPIO_PIN_SET);
}

static void dis_ce(struct drv_nrf24l01p *rd)
{
	assert(rd);
	HAL_GPIO_WritePin(rd->cePort, rd->cePin, GPIO_PIN_RESET);
}

//LSB first
static bool drv_nrf24l01p_write_spi(struct drv_nrf24l01p *rd, uint8_t reg, uint8_t *buf, uint16_t size, uint8_t *status)
{
	assert(rd);
	assert(buf);
	assert(size <= 32);

	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	reg |= DRV_NRF24L01P_CMD_W_REGISTER;

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, (status == NULL)?&(rd->statusReg):status, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, buf, rd->tmpBuf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	return true;
}

//LSB first
static bool drv_nrf24l01p_read_spi(struct drv_nrf24l01p *rd, uint8_t reg, uint8_t *buf, uint16_t size, uint8_t *status)
{
	assert((rd));
	assert((buf));
	assert(size <= 32);

	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	reg &= DRV_NRF24L01P_CMD_R_REGISTER;

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, (status == NULL)?&(rd->statusReg):status, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, rd->nopBuf, buf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	return true;
}

//ex) if you want to write 0bxxddddxx, data = 0bdddd, moreSigBitIdx = 5, lessSigBitIdx = 2
static bool drv_nrf24l01p_write_reg_byte(struct drv_nrf24l01p *rd, uint8_t reg, uint8_t data, uint8_t moreSigBitIdx, uint8_t lessSigBitIdx)
{
	uint8_t mask = 0;
	uint8_t tmpReg = 0;

	assert(lessSigBitIdx < moreSigBitIdx);
	assert(moreSigBitIdx <= 7);
	assert(lessSigBitIdx <= 7);

	for (int i = lessSigBitIdx; i <= moreSigBitIdx; i++) {
		mask |= _BV(i);
	}
	data <<= lessSigBitIdx;
	data &= mask;

	if (!drv_nrf24l01p_read_spi(rd, reg, &tmpReg, 1, NULL)) {
		return false;
	}

	tmpReg &= ~mask;
	tmpReg |= data;

	return drv_nrf24l01p_write_spi(rd, reg, &tmpReg, 1, NULL);
}

//ex) if moreSigBitIdx = 5 and lessSigBitIdx = 3, then data = (0bxxdddxxx >> lessSigBitIdx)
static bool drv_nrf24l01p_read_reg_byte(struct drv_nrf24l01p *rd, uint8_t reg, uint8_t *data, uint8_t moreSigBitIdx, uint8_t lessSigBitIdx)
{
	uint8_t mask = 0;
	uint8_t tmpReg = 0;

	assert(lessSigBitIdx < moreSigBitIdx);
	assert(moreSigBitIdx <= 7);
	assert(lessSigBitIdx <= 7);
	assert(data);

	for (int i = lessSigBitIdx; i <= moreSigBitIdx; i++) {
		mask |= _BV(i);
	}

	if (!drv_nrf24l01p_read_spi(rd, reg, &tmpReg, 1, NULL)) {
		return false;
	}

	tmpReg &= mask;
	tmpReg >>= lessSigBitIdx;

	*data = tmpReg;

	return true;
}

static bool drv_nrf24l01p_write_reg_bit(struct drv_nrf24l01p *rd, uint8_t reg, bool en, uint8_t bitIdx)
{
	uint8_t tmpReg = 0;

	assert(bitIdx <= 7);

	if (!drv_nrf24l01p_read_spi(rd, reg, &tmpReg, 1, NULL)) {
		return false;
	}

	tmpReg &= ~(1 << bitIdx);
	tmpReg |= en << bitIdx;

	return drv_nrf24l01p_write_spi(rd, reg, &tmpReg, 1, NULL);
}

static bool drv_nrf24l01p_read_reg_bit(struct drv_nrf24l01p *rd, uint8_t reg, bool *isEn, uint8_t bitIdx)
{
	uint8_t tmpReg = 0;

	assert(bitIdx <= 7);
	assert((isEn));

	if (!drv_nrf24l01p_read_spi(rd, reg, &tmpReg, 1, NULL)) {
		return false;
	}

	*isEn = tmpReg & _BV(bitIdx);

	return true;
}

bool drv_nrf24l01p_init(struct drv_nrf24l01p *rd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cePort, uint16_t cePin, GPIO_TypeDef *csPort, uint16_t csPin)
{
	assert(rd);
	assert(cePort);
	assert(csPort);
	assert(hspi);

	rd->hspi = hspi;
	rd->cePort = cePort;
	rd->cePin = cePin;
	rd->csPort = csPort;
	rd->csPin = csPin;

	dis_cs(rd);
	dis_ce(rd);

	for (int i = 0; i < 32; i++) {
		rd->nopBuf[i] = DRV_NRF24L01P_CMD_NOP;
	}

	return drv_nrf24l01p_en_power(rd, false);
}

bool drv_nrf24l01p_begin(struct drv_nrf24l01p *rd)
{
	bool isRx;
	uint8_t statusReg;


	if (!drv_nrf24l01p_get_primary_mode(rd, &isRx)) {
		return false;
	}

	if (isRx) {
		en_ce(rd);
	}

	drv_nrf24l01p_flush_rx_buf(rd);
	drv_nrf24l01p_flush_tx_buf(rd);
	drv_nrf24l01p_clear_irq(rd, true, true, true);

	if (!drv_nrf24l01p_read_status(rd, &statusReg)) {
		return false;
	}

	if (statusReg != 14) {
		return false;
	}

	return drv_nrf24l01p_en_power(rd, true);
}

bool drv_nrf24l01p_deinit(struct drv_nrf24l01p *rd)
{
	if (!drv_nrf24l01p_en_power(rd, false)) {
		return false;
	}

	dis_ce(rd);
	dis_cs(rd);

	return true;
}



//toggle ce pin. max 4ms
bool drv_nrf24l01p_write_tx_pld(struct drv_nrf24l01p *rd, uint8_t *pld, uint16_t size)
{
	const uint8_t reg = DRV_NRF24L01P_CMD_W_TX_PAYLOAD;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	assert(size >= 1);
	assert(size <= 32);
	assert((rd));
	assert((pld));

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, pld, rd->tmpBuf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	en_ce(rd);
	HAL_Delay(1);	//toggle cePin to transmit actually
	dis_ce(rd);

	return true;
}

//recommand to call drv_nrf24l01p_is_rx_empty() to check is fifo available
bool drv_nrf24l01p_read_rx_pld(struct drv_nrf24l01p *rd, uint8_t *pld, uint16_t size)
{
	const uint8_t reg = DRV_NRF24L01P_CMD_R_RX_PAYLOAD;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	assert(size >= 1);
	assert(size <= 32);
	assert((rd));
	assert(pld);

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, rd->nopBuf, pld, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	return true;
}

//indicate "don't transmit ack(specific packet)" to receiver.
//requirement : drv_nrf24l01p_en_dyn_ack()
bool drv_nrf24l01p_write_tx_pld_no_ack(struct drv_nrf24l01p *rd, uint8_t *pld, uint16_t size)
{
	const uint8_t reg = DRV_NRF24L01P_CMD_W_TX_PAYLOAD_NOACK;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	assert(size >= 1);
	assert(size <= 32);
	assert((rd));
	assert(pld);

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, pld, rd->tmpBuf, size, 1000);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}


	en_ce(rd);
	HAL_Delay(1);
	dis_ce(rd);

	return true;
}

//requirement : drv_nrf24l01p_en_ack_pld()
bool drv_nrf24l01p_write_ack_pld(struct drv_nrf24l01p *rd, uint8_t pipe, const uint8_t *pld, uint8_t size)
{
	uint8_t reg = DRV_NRF24L01P_CMD_W_ACK_PAYLOAD;
	HAL_StatusTypeDef spiStatus = HAL_ERROR;

	assert(size >= 1);
	assert(size <= 32);
	assert((rd));
	assert(pld);
	assert(pipe <= 5);

	reg |= pipe;

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, pld, rd->tmpBuf, size, 1000);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//don't call while transmitting
bool drv_nrf24l01p_reuse_tx_pld(struct drv_nrf24l01p *rd)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = DRV_NRF24L01P_CMD_REUSE_TX_PL;

	assert(rd);

	en_cs(rd);
	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//requirement : drv_nrf24l01p_set_dpl(), drv_nrf24l01p_en_dpl()
bool drv_nrf24l01p_read_pld_width(struct drv_nrf24l01p *rd, uint8_t *pldWidth)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = DRV_NRF24L01P_CMD_R_RX_PL_WID;

	assert(rd);
	assert(pldWidth);

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	if (spiStatus != HAL_OK) {
		dis_cs(rd);
		return false;
	}

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, pldWidth, 1, 300);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}	

	if (*pldWidth > 32) {
		drv_nrf24l01p_flush_rx_buf(rd);
		return false;
	}

	return true;
}

bool drv_nrf24l01p_read_status(struct drv_nrf24l01p *rd, uint8_t *status)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = DRV_NRF24L01P_CMD_NOP;

	assert(rd);
	assert(status);

	en_cs(rd);
	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, status, 1, 300);
	dis_cs(rd);

	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//use when  max_rt irq asserted
bool drv_nrf24l01p_flush_tx_buf(struct drv_nrf24l01p *rd)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = DRV_NRF24L01P_CMD_FLUSH_TX;

	assert((rd));

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

//use when drv_nrf24l01p_r_pld_width() return over 32
bool drv_nrf24l01p_flush_rx_buf(struct drv_nrf24l01p *rd)
{
	HAL_StatusTypeDef spiStatus = HAL_ERROR;
	const uint8_t reg = DRV_NRF24L01P_CMD_FLUSH_RX;

	assert((rd));

	en_cs(rd);

	spiStatus = HAL_SPI_TransmitReceive(rd->hspi, &reg, &rd->statusReg, 1, 300);
	dis_cs(rd);
	if (spiStatus != HAL_OK) {
		return false;
	}

	return true;
}

bool drv_nrf24l01p_en_irq(struct drv_nrf24l01p *rd, bool rx_dr, bool tx_ds, bool max_rt)
{
	return drv_nrf24l01p_write_reg_byte(rd, DRV_NRF24L01P_REG_CONFIG, (!rx_dr) << 2 | (!tx_ds) << 1| (!max_rt) , 6, 4);
}

bool drv_nrf24l01p_isen_irq(struct drv_nrf24l01p *rd, bool *rx_dr, bool *tx_ds, bool *max_rt)
{
	assert(rx_dr);
	assert(tx_ds);
	assert(max_rt);

	uint8_t configReg = 0;
	bool ret = false;

	ret = drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_CONFIG, &configReg, 6, 4);

	*rx_dr = !(configReg & _BV(2));
	*tx_ds = !(configReg & _BV(1));
	*max_rt = !(configReg & _BV(0));

	return ret;
}

//if irq asserted, use drv_nrf24l01p_clear_irq()
bool drv_nrf24l01p_read_irq(struct drv_nrf24l01p *rd, bool *rx_dr, bool *tx_ds, bool *max_rt)
{
	assert(rx_dr);
	assert(tx_ds);
	assert(max_rt);

	uint8_t configReg = 0;
	bool ret = false;

	ret = drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_STATUS, &configReg, 6, 4);

	*rx_dr = configReg & _BV(2);
	*tx_ds = configReg & _BV(1);
	*max_rt = configReg & _BV(0);


	return ret;
}

bool drv_nrf24l01p_clear_irq(struct drv_nrf24l01p *rd, bool rx_dr, bool tx_ds, bool max_rt)
{
	if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_STATUS, rx_dr, 6)) {
		return false;
	}

	if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_STATUS, tx_ds, 5)) {
		return false;
	}

	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_STATUS, max_rt, 4);
}

bool drv_nrf24l01p_set_crc_len(struct drv_nrf24l01p *rd, uint8_t crcLen)
{

	switch(crcLen) {
	case 1:
		if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, true, 3)) {
			return false;
		}
		if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, false, 2)) {
			return false;
		}
		break;
	case 2:
		if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, true, 3)) {
			return false;
		}
		if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, true, 2)) {
			return false;
		}
		break;
	case 0:
		if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, false, 3)) {
			return false;
		}
		break;
	default:
		return false;
	}

	return true;
}

bool drv_nrf24l01p_get_crc_len(struct drv_nrf24l01p *rd, uint8_t *crcLen)
{
	bool isEn_crc;
	bool is_crc_len2byte;

	assert(crcLen);

	if (!drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, &isEn_crc, 3)) {
		return false;
	}

	if (!drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, &is_crc_len2byte, 2)) {
		return false;
	}

	if (!isEn_crc) {
		*crcLen = 0;
	} else if (is_crc_len2byte) {
		*crcLen = 2;
	} else {
		*crcLen = 1;
	}

	return true;
}

bool drv_nrf24l01p_en_power(struct drv_nrf24l01p *rd, bool en)
{
	bool ret = drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, en, 1);
	HAL_Delay(2);
	return ret;
}

bool drv_nrf24l01p_isen_power(struct drv_nrf24l01p *rd, bool *isEn)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, isEn, 1);
}

bool drv_nrf24l01p_set_pmode(struct drv_nrf24l01p *rd, bool isRx)
{
	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, isRx, 0);
}

bool drv_nrf24l01p_get_primary_mode(struct drv_nrf24l01p *rd, bool *isRx)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_CONFIG, isRx, 0);
}

bool drv_nrf24l01p_en_auto_ack(struct drv_nrf24l01p *rd, uint8_t pipe, bool en)
{
	assert(pipe <= 5);
	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_EN_AA, en, pipe);
}

bool drv_nrf24l01p_isen_auto_ack(struct drv_nrf24l01p *rd, uint8_t pipe, bool *isEn)
{	
	assert(pipe <= 5);
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_EN_AA, isEn, pipe);
}

bool drv_nrf24l01p_en_rx_addr(struct drv_nrf24l01p *rd, uint8_t pipe, bool en)
{
	assert(pipe <= 5);
	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_EN_RXADDR, en, pipe);
}
bool drv_nrf24l01p_isen_rx_addr(struct drv_nrf24l01p *rd, uint8_t pipe, bool *isEn)
{
	assert(pipe <= 5);
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_EN_RXADDR, isEn, pipe);
}

bool drv_nrf24l01p_set_add_width(struct drv_nrf24l01p *rd, uint8_t add_width)
{
	assert(add_width >= 3 && add_width <= 5);

	return drv_nrf24l01p_write_reg_byte(rd, DRV_NRF24L01P_REG_SETUP_AW, add_width - 2, 1, 0);
}

bool drv_nrf24l01p_get_add_width(struct drv_nrf24l01p *rd, uint8_t *add_width)
{
	assert(add_width);

	if (!drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_SETUP_AW, add_width, 1, 0)) {
		return false;
	}

	*add_width += 2;
	if (*add_width > 5 || *add_width < 3) {
		return false;
	}

	return true;
}

bool drv_nrf24l01p_set_ard(struct drv_nrf24l01p *rd, uint16_t ard)
{
	assert(ard >= 250 && ard <= 4000);

	ard /= 250;
	ard--;

	return drv_nrf24l01p_write_reg_byte(rd, DRV_NRF24L01P_REG_SETUP_RETR, ard, 7, 4 );
}

bool drv_nrf24l01p_get_ard(struct drv_nrf24l01p *rd, uint16_t *ard)
{
	uint8_t tmp = 0;

	assert(ard);

	if (!drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_SETUP_RETR, &tmp, 7, 4)) {
		return false;
	}

	*ard = tmp;
	*ard += 1;
	*ard *= 250;

	if (*ard < 250 || *ard > 4000) {
		return false;
	}

	return true;
}

bool drv_nrf24l01p_set_arc(struct drv_nrf24l01p *rd, uint8_t arc)
{
	assert(arc <= 15);
	return drv_nrf24l01p_write_reg_byte(rd, DRV_NRF24L01P_REG_SETUP_RETR, arc, 3, 0);
}

bool drv_nrf24l01p_get_arc(struct drv_nrf24l01p *rd, uint8_t *arc)
{
	assert(arc);

	if (!drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_SETUP_RETR, arc, 3, 0)) {
		return false;
	}

	if (*arc > 15) {
		return false;
	}

	return true;
}

bool drv_nrf24l01p_set_channel(struct drv_nrf24l01p *rd, uint16_t mhz)
{
	assert(mhz >= 2400 && mhz <= 2525);
	return drv_nrf24l01p_write_reg_byte(rd, DRV_NRF24L01P_REG_RF_CH, mhz - 2400, 6, 0);
}

bool drv_nrf24l01p_get_channel(struct drv_nrf24l01p *rd, uint16_t *mhz)
{
	uint8_t tmp = 0;

	assert(mhz);

	if (!drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_RF_CH, &tmp, 6, 0)) {
		return false;
	}

	*mhz = tmp;
	*mhz += 2400;

	if (*mhz < 2400 || *mhz > 2525)
		return false;

	return true;
}

bool drv_nrf24l01p_set_data_rate(struct drv_nrf24l01p *rd, enum drv_nrf24l01p_data_rate data_rate)
{
	if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_RF_SETUP, false, 5)) {
		return false;
	}

	if (!drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_RF_SETUP, false, 3)) {
		return false;
	}

	if (data_rate == DRV_NRF24L01P_2MBPS) {
		return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_RF_SETUP, true, 3);
	} else if(data_rate == DRV_NRF24L01P_250KBPS) {
		return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_RF_SETUP, true, 5);
	} else if(data_rate != DRV_NRF24L01P_1MBPS) {
		return false;
	}

	return true;
}

bool drv_nrf24l01p_get_data_rate(struct drv_nrf24l01p *rd, enum drv_nrf24l01p_data_rate *data_rate)
{
	bool lowBit = 0;
	bool highBit = 0;

	assert(data_rate);

	if (!drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_RF_SETUP, &lowBit, 5)) {
		return false;
	}

	if (!drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_RF_SETUP, &highBit, 3)) {
		return false;
	}

	if (lowBit && highBit) {
		return false;
	} else if (lowBit) {
		*data_rate = DRV_NRF24L01P_250KBPS;
	} else if (highBit) {
		*data_rate = DRV_NRF24L01P_2MBPS;
	} else {
		*data_rate = DRV_NRF24L01P_1MBPS;
	}

	return true;
}

bool drv_nrf24l01p_set_paPower(struct drv_nrf24l01p *rd,  enum drv_nrf24l01p_pa_power paPower)
{
	return drv_nrf24l01p_write_reg_byte(rd, DRV_NRF24L01P_REG_RF_SETUP, paPower, 2, 1);
}

bool drv_nrf24l01p_get_paPower(struct drv_nrf24l01p *rd,  enum drv_nrf24l01p_pa_power *paPower)
{
	return drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_RF_SETUP, paPower, 2, 1);
}

bool drv_nrf24l01p_read_pipeNum(struct drv_nrf24l01p *rd, uint8_t *pipe)
{
	assert(pipe);

	if (!drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_STATUS, pipe, 3, 1)) {
		return false;
	}

	if (*pipe > 5) {
		return false;
	}

	return true;
}

bool drv_nrf24l01p_is_tx_bufFull(struct drv_nrf24l01p *rd, bool *is_full)
{	
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_STATUS, is_full, 0);
}

bool drv_nrf24l01p_is_tx_bufEmpty(struct drv_nrf24l01p *rd, bool *is_empty)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_FIFO_STATUS, is_empty, 4);
}

bool drv_nrf24l01p_is_rx_bufFull(struct drv_nrf24l01p *rd, bool *is_full)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_FIFO_STATUS, is_full, 1);
}

bool drv_nrf24l01p_is_rx_bufEmpty(struct drv_nrf24l01p *rd, bool *is_empty)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_FIFO_STATUS, is_empty, 0);
}

//packet loss count. +1 plost_cnt when max_rt irq asserted
//if plostCnt == 15 then transmit operation halted
//write value in RF_CH register with drv_nrf24l01p_set_channel() to reset plos_cnt.
bool drv_nrf24l01p_read_plosCnt(struct drv_nrf24l01p *rd, uint8_t *plosCnt)
{
	bool ret = false;

	assert(plosCnt);

	ret = drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_OBSERVE_TX, plosCnt, 7, 4);
	if (!ret) {
		return ret;
	}

	if (*plosCnt > 15) {
		return false;
	}

	return ret;
}

bool drv_nrf24l01p_read_arc(struct drv_nrf24l01p *rd, uint8_t *arc)
{
	bool ret = false;

	assert(arc);

	ret = drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_OBSERVE_TX, arc, 3, 0);
	if (!ret) {
		return ret;
	}

	if (*arc > 15) {
		return false;
	}

	return ret;
}

bool drv_nrf24l01p_set_rx_addr(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width)
{
	bool ret = false;
	uint8_t read_addr[5] = {0,};

	assert(rxAddr);
	assert(width >= 3 && width <= 5);
	assert(pipe <= 5);

	if (pipe == 0 || pipe == 1) {
		return drv_nrf24l01p_write_spi(rd, DRV_NRF24L01P_REG_RX_ADDR_P0 + pipe, rxAddr, width, NULL);
	}

	ret = drv_nrf24l01p_read_spi(rd, DRV_NRF24L01P_REG_RX_ADDR_P1, read_addr, width, NULL);
	if (!ret) {
		return ret;
	}

	for (int i = 1; i < width; i++) {
		if (read_addr[i] != rxAddr[i]) {
			return false;
		}
	}

	return drv_nrf24l01p_write_spi(rd, DRV_NRF24L01P_REG_RX_ADDR_P0 + pipe, rxAddr, 1, NULL);
}

bool drv_nrf24l01p_get_rx_addr(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width)
{
	assert(width >= 3 && width <= 5);
	assert(pipe <= 5);

	if (pipe == 0) {
		return drv_nrf24l01p_read_spi(rd, DRV_NRF24L01P_REG_RX_ADDR_P0, rxAddr, width, NULL);
	}

	if (!drv_nrf24l01p_read_spi(rd, DRV_NRF24L01P_REG_RX_ADDR_P1, rxAddr, width, NULL)) {
		return false;
	}

	if (pipe != 1) {
		return drv_nrf24l01p_read_spi(rd, DRV_NRF24L01P_REG_RX_ADDR_P0 + pipe, rxAddr, 1, NULL);
	}

	return true;
}

bool drv_nrf24l01p_set_txAddr(struct drv_nrf24l01p *rd, uint8_t *txAddr, uint8_t width)
{
	assert(width >= 3 && width <= 5);
	return drv_nrf24l01p_write_spi(rd, DRV_NRF24L01P_REG_TX_ADDR, txAddr, width, NULL);
}

bool drv_nrf24l01p_get_txAddr(struct drv_nrf24l01p *rd, uint8_t *txAddr, uint8_t width)
{
	assert(width >= 3 && width <= 5);
	return drv_nrf24l01p_read_spi(rd, DRV_NRF24L01P_REG_TX_ADDR, txAddr, width, NULL);
}

bool drv_nrf24l01p_set_rx_pld_width(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t pldWidth)
{
	assert(pldWidth <= 32);
	assert(pipe <= 5);

	return drv_nrf24l01p_write_reg_byte(rd, DRV_NRF24L01P_REG_RX_PW_P0 + pipe, pldWidth, 5, 0);
}

bool drv_nrf24l01p_get_rx_pld_width(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t *pldWidth)
{
	bool ret = false;

	assert(pipe <= 5);

	ret = drv_nrf24l01p_read_reg_byte(rd, DRV_NRF24L01P_REG_RX_PW_P0 + pipe, pldWidth, 5, 0);
	if (!ret) {
		return ret;
	}

	if (*pldWidth > 32) {
		drv_nrf24l01p_flush_rx_buf(rd);
		return false;
	}

	return ret;
}

//requirement: drv_nrf24l01p_en_auto_ack
bool drv_nrf24l01p_set_dpl(struct drv_nrf24l01p *rd, uint8_t pipe, bool en)
{
	assert(pipe <= 5);
	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_DYNPD, en, pipe);
}

bool drv_nrf24l01p_get_dpl(struct drv_nrf24l01p *rd, uint8_t pipe, bool *isEn)
{
	assert(pipe <= 5);
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_DYNPD, isEn, pipe);

}

bool drv_nrf24l01p_en_dpl(struct drv_nrf24l01p *rd, bool en)
{
	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_FEATURE, en, 2);
}

bool drv_nrf24l01p_isen_dpl(struct drv_nrf24l01p *rd, bool *isEn)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_FEATURE, isEn, 2);
}

//requirement : drv_nrf24l01p_set_dpl(), drv_nrf24l01p_en_dpl()
//specific pipe that dpl feature on.
bool drv_nrf24l01p_en_ack_pld(struct drv_nrf24l01p *rd, bool en)
{
	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_FEATURE, en, 1);
}

bool drv_nrf24l01p_isen_ack_pld(struct drv_nrf24l01p *rd, bool *isEn)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_FEATURE, isEn, 1);
}

bool drv_nrf24l01p_en_dyn_ack(struct drv_nrf24l01p *rd, bool en)
{
	return drv_nrf24l01p_write_reg_bit(rd, DRV_NRF24L01P_REG_FEATURE, en, 0);
}

bool drv_nrf24l01p_isen_dyn_ack(struct drv_nrf24l01p *rd, bool *isEn)
{
	return drv_nrf24l01p_read_reg_bit(rd, DRV_NRF24L01P_REG_FEATURE, isEn, 0);
}


bool drv_nrf24l01p_set_arduino_style(struct drv_nrf24l01p *rd)
{

	if (!drv_nrf24l01p_set_arc(rd, 15)) {
		return false;
	}

	if (!drv_nrf24l01p_set_ard(rd, 250 * 5)) {
		return false;
	}

	if (!drv_nrf24l01p_set_data_rate(rd, DRV_NRF24L01P_1MBPS)) {
		return false;
	}

	if (!drv_nrf24l01p_en_ack_pld(rd, false)) {
		return false;
	}

	if (!drv_nrf24l01p_en_dpl(rd, false)) {
		return false;
	}

	if (!drv_nrf24l01p_en_dyn_ack(rd, false)) {
		return false;
	}

	for (int i = 0; i < 6; i++) {
		if (!drv_nrf24l01p_set_dpl(rd, i, false)) {
			return false;
		}
	}
	for (int i = 0; i < 6; i++) {
		if (!drv_nrf24l01p_en_auto_ack(rd, i, true)) {
			return false;
		}
	}

	if (!drv_nrf24l01p_en_rx_addr(rd, 0, true)) {
		return false;
	}

	if (!drv_nrf24l01p_en_rx_addr(rd, 1, true)) {
		return false;
	}

	for (int i = 0; i < 6; i++) {
		if (!drv_nrf24l01p_set_rx_pld_width(rd, i, 32)) {
			return false;
		}
	}

	if (!drv_nrf24l01p_set_add_width(rd, 5)) {
		return false;
	}

	if (!drv_nrf24l01p_set_channel(rd, 2476)) {
		return false;
	}

	if (!drv_nrf24l01p_clear_irq(rd, false, false, false)) {
		return false;
	}

	if (!drv_nrf24l01p_flush_tx_buf(rd)) {
		return false;
	}

	if (!drv_nrf24l01p_flush_rx_buf(rd)) {
		return false;
	}

	if (!drv_nrf24l01p_set_crc_len(rd, 2)) {
		return false;
	}

	return true;
}
