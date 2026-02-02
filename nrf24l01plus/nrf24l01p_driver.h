/*
 * nrf24l01p_driver.h
 *
 *  Created on: Apr 26, 2025
 *      Author: coder0908
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define DRV_NRF24L01P_MAX_PLD_WIDTH 32

struct drv_nrf24l01p {
	GPIO_TypeDef 	*cePort;
	uint16_t  	cePin;

	GPIO_TypeDef *csPort;
	uint16_t csPin;

	SPI_HandleTypeDef *hspi;

	uint8_t statusReg;
	uint8_t tmpBuf[32];
	uint8_t nopBuf[32];
};

enum drv_nrf24l01p_pa_power {
	DRV_NRF24L01P_PA_M18DBM = 0,
	DRV_NRF24L01P_PA_M12DBM,
	DRV_NRF24L01P_PA_M6DBM,
	DRV_NRF24L01P_PA_0DBM
};

enum drv_nrf24l01p_data_rate {
	DRV_NRF24L01P_1MBPS = 0,
	DRV_NRF24L01P_2MBPS,
	DRV_NRF24L01P_250KBPS
};

bool drv_nrf24l01p_init(struct drv_nrf24l01p *rd, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cePort, uint16_t cePin, GPIO_TypeDef *csPort, uint16_t csPin);
bool drv_nrf24l01p_deinit(struct drv_nrf24l01p *rd);

bool drv_nrf24l01p_begin(struct drv_nrf24l01p *rd);

bool drv_nrf24l01p_write_txPld(struct drv_nrf24l01p *rd, uint8_t *txPld, uint16_t size);
bool drv_nrf24l01p_read_rxPld(struct drv_nrf24l01p *rd, uint8_t *rxPld, uint16_t size);
bool drv_nrf24l01p_write_txPldNoAck(struct drv_nrf24l01p *rd, uint8_t *txPld, uint16_t size);
bool drv_nrf24l01p_write_ackPld(struct drv_nrf24l01p *rd, uint8_t pipe, const uint8_t *ackPld, uint8_t size);

bool drv_nrf24l01p_reuse_txPld(struct drv_nrf24l01p *rd);

bool drv_nrf24l01p_read_pldWidth(struct drv_nrf24l01p *rd, uint8_t *pldWidth);
bool drv_nrf24l01p_read_status(struct drv_nrf24l01p *rd, uint8_t *status);

bool drv_nrf24l01p_flush_txBuf(struct drv_nrf24l01p *rd);
bool drv_nrf24l01p_flush_rxBuf(struct drv_nrf24l01p *rd);

bool drv_nrf24l01p_en_irq(struct drv_nrf24l01p *rd, bool rx_dr, bool tx_ds, bool max_rt);
bool drv_nrf24l01p_isEn_irq(struct drv_nrf24l01p *rd, bool *rx_dr, bool *tx_ds, bool *max_rt);

bool drv_nrf24l01p_read_irq(struct drv_nrf24l01p *rd, bool *rx_dr, bool *tx_ds, bool *max_rt);
bool drv_nrf24l01p_clear_irq(struct drv_nrf24l01p *rd, bool rx_dr, bool tx_ds, bool max_rt);

bool drv_nrf24l01p_set_crcLen(struct drv_nrf24l01p *rd, uint8_t crcLen);
bool drv_nrf24l01p_get_crcLen(struct drv_nrf24l01p *rd, uint8_t *crcLen);

bool drv_nrf24l01p_en_power(struct drv_nrf24l01p *rd, bool en);
bool drv_nrf24l01p_isEn_power(struct drv_nrf24l01p *rd, bool *isEn);

bool drv_nrf24l01p_set_pmode(struct drv_nrf24l01p *rd, bool isRx);
bool drv_nrf24l01p_get_pmode(struct drv_nrf24l01p *rd, bool *isRx);

bool drv_nrf24l01p_en_autoAck(struct drv_nrf24l01p *rd, uint8_t pipe, bool en);
bool drv_nrf24l01p_isEn_autoAck(struct drv_nrf24l01p *rd, uint8_t pipe, bool *isEn);

bool drv_nrf24l01p_en_rxAddr(struct drv_nrf24l01p *rd, uint8_t pipe, bool en);
bool drv_nrf24l01p_isEn_rxAddr(struct drv_nrf24l01p *rd, uint8_t pipe, bool *isEn);

bool drv_nrf24l01p_set_addrWidth(struct drv_nrf24l01p *rd, uint8_t addrWidth);
bool drv_nrf24l01p_get_addrWidth(struct drv_nrf24l01p *rd, uint8_t *addrWidth);

bool drv_nrf24l01p_set_ard(struct drv_nrf24l01p *rd, uint16_t ard);
bool drv_nrf24l01p_get_ard(struct drv_nrf24l01p *rd, uint16_t *ard);

bool drv_nrf24l01p_set_arc(struct drv_nrf24l01p *rd, uint8_t arc);
bool drv_nrf24l01p_get_arc(struct drv_nrf24l01p *rd, uint8_t *arc);

bool drv_nrf24l01p_set_channel(struct drv_nrf24l01p *rd, uint16_t mhz);
bool drv_nrf24l01p_get_channel(struct drv_nrf24l01p *rd, uint16_t *mhz);

bool drv_nrf24l01p_set_dataRate(struct drv_nrf24l01p *rd, enum drv_nrf24l01p_data_rate dataRate);
bool drv_nrf24l01p_get_dataRate(struct drv_nrf24l01p *rd, enum drv_nrf24l01p_data_rate *dataRate);

bool drv_nrf24l01p_set_paPower(struct drv_nrf24l01p *rd, enum drv_nrf24l01p_pa_power paPower);
bool drv_nrf24l01p_get_paPower(struct drv_nrf24l01p *rd, enum drv_nrf24l01p_pa_power *paPower);

bool drv_nrf24l01p_read_pipeNum(struct drv_nrf24l01p *rd, uint8_t *pipe);

bool drv_nrf24l01p_is_txBufFull(struct drv_nrf24l01p *rd, bool *isFull);
bool drv_nrf24l01p_is_rxBufFull(struct drv_nrf24l01p *rd, bool *isFull);

bool drv_nrf24l01p_is_rxBufEmpty(struct drv_nrf24l01p *rd, bool *isEmpty);
bool drv_nrf24l01p_is_txBufEmpty(struct drv_nrf24l01p *rd, bool *isEmpty);

bool drv_nrf24l01p_read_plosCnt(struct drv_nrf24l01p *rd, uint8_t *plosCnt);
bool drv_nrf24l01p_read_arc(struct drv_nrf24l01p *rd, uint8_t *arc);

bool drv_nrf24l01p_set_rxAddr(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width);
bool drv_nrf24l01p_get_rxAddr(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t *rxAddr, uint8_t width);

bool drv_nrf24l01p_set_txAddr(struct drv_nrf24l01p *rd, uint8_t *tx_addr, uint8_t width);
bool drv_nrf24l01p_get_txAddr(struct drv_nrf24l01p *rd, uint8_t *tx_addr, uint8_t width);

bool drv_nrf24l01p_set_rxPldWidth(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t pldWidth);
bool drv_nrf24l01p_get_rxPldWidth(struct drv_nrf24l01p *rd, uint8_t pipe, uint8_t *pldWidth);

bool drv_nrf24l01p_set_dpl(struct drv_nrf24l01p *rd, uint8_t pipe, bool en);
bool drv_nrf24l01p_get_dpl(struct drv_nrf24l01p *rd, uint8_t pipe, bool *isEn);
bool drv_nrf24l01p_en_dpl(struct drv_nrf24l01p *rd, bool en);
bool drv_nrf24l01p_isEn_dpl(struct drv_nrf24l01p *rd, bool *en);

bool drv_nrf24l01p_en_ackPld(struct drv_nrf24l01p *rd, bool en);
bool drv_nrf24l01p_isEn_ackPld(struct drv_nrf24l01p *rd, bool *isEn);

bool drv_nrf24l01p_en_dynAck(struct drv_nrf24l01p *rd, bool en);
bool drv_nrf24l01p_isEn_dynAck(struct drv_nrf24l01p *rd, bool *isEn);

bool drv_nrf24l01p_set_arduinoStyle(struct drv_nrf24l01p *rd);
