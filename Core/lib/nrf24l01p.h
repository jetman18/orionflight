  
/*
 *  nrf24l01_plus.h
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__


#include "spi.h"    // header from stm32cubemx code generate
#include <stdbool.h>


/* nRF24L01+ typedefs */
typedef uint8_t count;
typedef uint8_t widths;
typedef uint8_t length;
typedef uint16_t delay;
typedef uint16_t channel;

typedef enum
{
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
} air_data_rate;

typedef enum
{
    _0dBm  = 3,
    _6dBm  = 2,
    _12dBm = 1,
    _18dBm = 0
} output_power;


/* Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps);
void nrf24l01p_tx_init(channel MHz, air_data_rate bps);

void nrf24l01p_rx_receive(uint8_t* rx_payload);
void nrf24l01p_tx_transmit(uint8_t* tx_payload);

// Check tx_ds or max_rt
void nrf24l01p_tx_irq();  


/* Sub Functions */
void nrf24l01p_reset();

void nrf24l01p_prx_mode();
void nrf24l01p_ptx_mode();

void nrf24l01p_power_up();
void nrf24l01p_power_down();

uint8_t nrf24l01p_get_status();
uint8_t nrf24l01p_get_fifo_status();

// Static payload lengths
void nrf24l01p_rx_set_payload_widths(widths bytes);

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload);
uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload);

void nrf24l01p_flush_rx_fifo();
void nrf24l01p_flush_tx_fifo();

// Clear IRQ pin. Change LOW to HIGH
void nrf24l01p_clear_rx_dr();
void nrf24l01p_clear_tx_ds();
void nrf24l01p_clear_max_rt();

void nrf24l01p_set_rf_channel(channel MHz);
void nrf24l01p_set_rf_tx_output_power(output_power dBm);
void nrf24l01p_set_rf_air_data_rate(air_data_rate bps);

void nrf24l01p_set_crc_length(length bytes);
void nrf24l01p_set_address_widths(widths bytes);
void nrf24l01p_auto_retransmit_count(count cnt);
void nrf24l01p_auto_retransmit_delay(delay us);


#endif /* __NRF24L01P_H__ */
