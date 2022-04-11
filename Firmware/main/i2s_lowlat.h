/*
 * i2s_lowlat.h - Low Latency I2S for realtime synthesis
 * 02-07-2022 E. Brombaugh
 *
 * Based on esp-idf V5.0 I2S drivers
 *
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/i2s_periph.h"
#include "soc/rtc_periph.h"
#include "soc/soc_caps.h"
#include "hal/i2s_types.h"
#include "esp_intr_alloc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2S_PIN_NO_CHANGE (-1) /*!< Use in i2s_pin_config_t for pins which should not be changed */

/**
 * @brief I2S port number, the max port number is (I2S_NUM_MAX -1).
 */
typedef enum {
    I2S_NUM_0 = 0,                 /*!< I2S port 0 */
#if SOC_I2S_NUM > 1
    I2S_NUM_1 = 1,                 /*!< I2S port 1 */
#endif
    I2S_NUM_MAX,                   /*!< I2S port max */
} i2s_port_t;

#if SOC_I2S_SUPPORTS_PCM
/**
 * @brief I2S PCM configuration
 *
 */
typedef struct {
    i2s_pcm_compress_t  pcm_type;       /*!< I2S PCM a/u-law decompress or compress type */
} i2s_pcm_cfg_t;
#endif

/**
 * @brief I2S pin number for i2s_set_pin
 *
 */
typedef struct {
    int mck_io_num;     /*!< MCK in out pin. Note that ESP32 supports setting MCK on GPIO0/GPIO1/GPIO3 only*/
    int bck_io_num;     /*!< BCK in out pin*/
    int ws_io_num;      /*!< WS in out pin*/
    int data_out_num;   /*!< DATA out pin*/
    int data_in_num;    /*!< DATA in pin*/
	int tx_diag_out_num;/*!< TX DIAG out pin */
	int rx_diag_out_num;/*!< RX DIAG out pin */
} i2s_pin_config_t;

/**
 * @brief callback functions for TX and RX
 *
 */
typedef void (*i2s_callback_t)(const volatile uint8_t **buf, uint32_t len);

/**
 * @brief I2S driver configuration parameters
 *
 */
typedef struct {

    i2s_mode_t              mode;                       /*!< I2S work mode */
    uint32_t                sample_rate;                /*!< I2S sample rate */
    i2s_bits_per_sample_t   bits_per_sample;            /*!< I2S sample bits in one channel */
    i2s_channel_fmt_t       channel_format;             /*!< I2S channel format.*/
    i2s_comm_format_t       communication_format;       /*!< I2S communication format */
    int                     intr_alloc_flags;           /*!< Flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info */
    union {
        int dma_frame_num;                              /*!< Number of frames for one-time sampling. The frame here means the total data from all the channels in a WS cycle */
        int dma_buf_len __attribute__((deprecated));    /*!< This is an alias to 'dma_frame_num' for backward compatibility */
    };

    bool                    use_apll;                   /*!< I2S using APLL as main I2S clock, enable it to get accurate clock */
    int                     fixed_mclk;                 /*!< I2S using fixed MCLK output. If use_apll = true and fixed_mclk > 0, then the clock output for i2s is fixed and equal to the fixed_mclk value. If fixed_mclk set, mclk_multiple won't take effect */
    i2s_mclk_multiple_t     mclk_multiple;              /*!< The multiple of I2S master clock(MCLK) to sample rate */
    i2s_bits_per_chan_t     bits_per_chan;              /*!< I2S total bits in one channel， only take effect when larger than 'bits_per_sample', default '0' means equal to 'bits_per_sample' */
    i2s_callback_t			tx_proc_cb ; 				/*!< Invoked when TX needs buffer update */
    i2s_callback_t			rx_proc_cb ; 				/*!< Invoked when RX needs buffer update */

#if SOC_I2S_SUPPORTS_TDM
    i2s_channel_t           chan_mask;                  /*!< I2S active channel bit mask, set value in `i2s_channel_t` to enable specific channel, the bit map of active channel can not exceed (0x1<<total_chan). */
    uint32_t                total_chan;                 /*!< I2S Total number of channels. If it is smaller than the biggest active channel number, it will be set to this number automatically. */
    bool                    left_align;                 /*!< Set to enable left alignment */
    bool                    big_edin;                   /*!< Set to enable big edin */
    bool                    bit_order_msb;              /*!< Set to enable msb order */
    bool                    skip_msk;                   /*!< Set to enable skip mask. If it is enabled, only the data of the enabled channels will be sent, otherwise all data stored in DMA TX buffer will be sent */
#endif // SOC_I2S_SUPPORTS_TDM

} i2s_driver_config_t;

typedef i2s_driver_config_t i2s_config_t;       // for backward compatible
typedef intr_handle_t i2s_isr_handle_t;         // for backward compatible


/**
 * @brief Set I2S pin number
 *
 * @note
 * The I2S peripheral output signals can be connected to multiple GPIO pads.
 * However, the I2S peripheral input signal can only be connected to one GPIO pad.
 *
 * @param   i2s_num     I2S port number
 *
 * @param   pin         I2S Pin structure, or NULL to set 2-channel 8-bit internal DAC pin configuration (GPIO25 & GPIO26)
 *
 * Inside the pin configuration structure, set I2S_PIN_NO_CHANGE for any pin where
 * the current configuration should not be changed.
 *
 * @note if *pin is set as NULL, this function will initialize both of the built-in DAC channels by default.
 *       if you don't want this to happen and you want to initialize only one of the DAC channels, you can call i2s_set_dac_mode instead.
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL            IO error
 */
esp_err_t i2s_set_pin(i2s_port_t i2s_num, const i2s_pin_config_t *pin);

/**
 * @brief Install and start I2S driver.
 *
 * @param i2s_num         I2S port number
 *
 * @param i2s_config      I2S configurations - see i2s_config_t struct
 *
 * This function must be called before any I2S driver read/write operations.
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM      Out of memory
 *     - ESP_ERR_INVALID_STATE  Current I2S port is in use
 */
esp_err_t i2s_driver_install(i2s_port_t i2s_num, const i2s_config_t *i2s_config);

/**
 * @brief Uninstall I2S driver.
 *
 * @param i2s_num  I2S port number
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_INVALID_STATE I2S port has been uninstalled by others (e.g. LCD i80)
 */
esp_err_t i2s_driver_uninstall(i2s_port_t i2s_num);

/**
 * @brief Set sample rate used for I2S RX and TX.
 *
 * The bit clock rate is determined by the sample rate and i2s_config_t configuration parameters (number of channels, bits_per_sample).
 *
 * `bit_clock = rate * (number of channels) * bits_per_sample`
 *
 * @param i2s_num  I2S port number
 *
 * @param rate I2S sample rate (ex: 8000, 44100...)
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM      Out of memory
 */
esp_err_t i2s_set_sample_rates(i2s_port_t i2s_num, uint32_t rate);

/**
 * @brief Stop I2S driver
 *
 * There is no need to call i2s_stop() before calling i2s_driver_uninstall().
 *
 * Disables I2S TX/RX, until i2s_start() is called.
 *
 * @param i2s_num  I2S port number
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2s_stop(i2s_port_t i2s_num);

/**
 * @brief Start I2S driver
 *
 * It is not necessary to call this function after i2s_driver_install() (it is started automatically), however it is necessary to call it after i2s_stop().
 *
 *
 * @param i2s_num  I2S port number
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2s_start(i2s_port_t i2s_num);

#if SOC_I2S_SUPPORTS_PCM
/**
 * @brief Configure I2S a/u-law decompress or compress
 *
 * @note  This function should be called after i2s driver installed
 *        Only take effecttive when the i2s 'communication_format' is set to 'I2S_COMM_FORMAT_STAND_PCM_SHORT' or 'I2S_COMM_FORMAT_STAND_PCM_LONG'
 *
 * @param i2s_num  I2S port number
 *
 * @param pcm_cfg  including mode selection and a/u-law decompress or compress configuration paramater
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t i2s_pcm_config(i2s_port_t i2s_num, const i2s_pcm_cfg_t *pcm_cfg);
#endif

/**
 * @brief Set clock & bit width used for I2S RX and TX.
 *
 * Similar to i2s_set_sample_rates(), but also sets bit width.
 *
 * 1. stop i2s;
 * 2. calculate mclk, bck, bck_factor
 * 3. malloc dma buffer;
 * 4. start i2s
 *
 * @param i2s_num  I2S port number
 *
 * @param rate I2S sample rate (ex: 8000, 44100...)
 *
 * @param bits_cfg I2S bits configuration
 *             the low 16 bits is for data bits per sample in one channel (see 'i2s_bits_per_sample_t')
 *             the high 16 bits is for total bits in one channel (see 'i2s_bits_per_chan_t')
 *             high 16bits =0 means same as the bits per sample.
 *
 * @param ch I2S channel, (I2S_CHANNEL_MONO, I2S_CHANNEL_STEREO or specific channel in TDM mode)
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_ERR_NO_MEM      Out of memory
 */
esp_err_t i2s_set_clk(i2s_port_t i2s_num, uint32_t rate, uint32_t bits_cfg, i2s_channel_t ch);

/**
 * @brief get clock set on particular port number.
 *
 * @param i2s_num  I2S port number
 *
 * @return
 *     - actual clock set by i2s driver
 */
float i2s_get_clk(i2s_port_t i2s_num);

#ifdef __cplusplus
}
#endif
