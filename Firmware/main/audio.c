/*
 * audio.c - audio via I2S.
 * 03-9-22 E. Brombaugh
 */
 
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "audio.h"
#include "i2s_lowlat.h"
#include "eb_adc.h"
#include "fx.h"

#define I2S_NUM         (0)
#define I2S_BCK_IO      (GPIO_NUM_4)
#define I2S_WS_IO       (GPIO_NUM_2)
#define I2S_DO_IO       (GPIO_NUM_3)
#define I2S_DI_IO       (GPIO_NUM_1)	// (-1)
#define I2S_MCK_IO      (GPIO_NUM_12)
#define I2S_NUM_BUFS	2

static const char* TAG = "audio";
int16_t audio_sl[4];
uint64_t audio_load[3];
int16_t audio_mute_state, audio_mute_cnt;
int16_t *rxbuf = NULL;
int16_t prc[2*FRAMESZ];

/*
 * NOTE: ESP32 HW interleaves stereo R/L/R/L with R @ index 0
 * (opposite of most other systems)
 */
 
/*
 * signal level calc
 */
void IRAM_ATTR level_calc(int16_t sig, int16_t *level)
{
	/* rectify */
	sig = (sig < 0) ? -sig : sig;

	/* peak hold - externally reset */
	if(*level < sig)
		*level = sig;
}

/*
 * Audio processing callbacks
 */
void IRAM_ATTR audio_tx_proc(const volatile uint8_t **buf, uint32_t len)
{
	uint8_t i;
	int32_t wet, dry, mix;
	
	/* skip processing if no rx */
	if(rxbuf == NULL)
		return;
	
	/* update start time for load calcs */
	audio_load[2] = audio_load[0];
	audio_load[0] = esp_timer_get_time();
	
	/* init buffer ptrs */
	int16_t *src = rxbuf;
	int16_t *dst = (int16_t *)(*buf);
	
	len >>= 2;	// len input is in bytes - we need stereo 16-bit samples
	
	/* check input levels */
	for(i=0;i<len;i++)
	{
		level_calc(src[2*i], &audio_sl[0]);
		level_calc(src[2*i+1], &audio_sl[1]);
	}
	
	/* process the selected algorithm */
	fx_proc(prc, src, len);
	
	/* set W/D mix gain */	
	wet = adc_val[0];
	dry = 0xfff - wet;
	
	/* handle output */
	for(i=0;i<len;i++)
	{
		/* W/D with saturation */
		mix = prc[2*i] * wet + src[2*i] * dry;
		dst[2*i] = dsp_ssat16(mix>>12);
		mix = prc[2*i+1] * wet + src[2*i+1] * dry;
		dst[2*i+1] = dsp_ssat16(mix>>12);
		
		/* handle muting */
		switch(audio_mute_state)
		{
			case 0:
				/* pass thru and wait for foreground to force a transition */
				break;
			
			case 1:
				/* transition to mute state */
				mix = (dst[2*i] * audio_mute_cnt);
				dst[2*i] = dsp_ssat16(mix>>9);
				mix = (dst[2*i+1] * audio_mute_cnt);
				dst[2*i+1] = dsp_ssat16(mix>>9);
				audio_mute_cnt--;
				if(audio_mute_cnt == 0)
					audio_mute_state = 2;
				break;
				
			case 2:
				/* mute and wait for foreground to force a transition */
				dst[2*i] = 0;
				dst[2*i+1] = 0;
				break;
			
			case 3:
				/* transition to unmute state */
				mix = (dst[2*i] * audio_mute_cnt);
				dst[2*i] = dsp_ssat16(mix>>9);
				mix = (dst[2*i+1] * audio_mute_cnt);
				dst[2*i+1] = dsp_ssat16(mix>>9);
				audio_mute_cnt++;
				if(audio_mute_cnt == 512)
				{
					audio_mute_state = 0;
					audio_mute_cnt = 0;
				}
				break;
				
			default:
				/* go to legal state */
				audio_mute_state = 0;
				break;
		}
	
		/* check output levels */
		level_calc(dst[2*i], &audio_sl[2]);
		level_calc(dst[2*i+1], &audio_sl[3]);
	}
	
	/* update end timer */
	audio_load[1] = esp_timer_get_time();
}

void IRAM_ATTR audio_rx_proc(const volatile uint8_t **buf, uint32_t len)
{
	rxbuf = (int16_t *)(*buf);
}

/*
 * initializer
 */
esp_err_t audio_init(void)
{
	/* init fx */
	fx_init();
	
	/* init state */
	audio_sl[0] = audio_sl[1] = audio_sl[2] = audio_sl[3] = 0;
	audio_load[0] = audio_load[1] = audio_load[2] = 0;
	audio_mute_state = 2;	// start up  muted
	audio_mute_cnt = 0;
	
	/* init I2S stuff */
    ESP_LOGI(TAG, "Initializing I2S Low Latency driver");
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_frame_num = FRAMESZ,
        .use_apll = true,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
		.tx_proc_cb = audio_tx_proc,
		.rx_proc_cb = audio_rx_proc,
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_MCK_IO,
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_DO_IO,
        .data_in_num = I2S_DI_IO        //Not used
    };
    i2s_driver_install(I2S_NUM, &i2s_config);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_set_clk(I2S_NUM, SAMPLE_RATE, 16, 2);
	
	return ESP_OK;
}

/*
 * internal soft mute
 */
void audio_mute(uint8_t enable)
{
	if((audio_mute_state == 0) && (enable == 1))
	{
		audio_mute_cnt = 512;
		audio_mute_state = 1;
		while(audio_mute_state != 2)
		{
			vTaskDelay(1);
		}
	}
	else if((audio_mute_state == 2) && (enable == 0))
	{
		audio_mute_cnt = 0;
		audio_mute_state = 3;
		while(audio_mute_state != 0)
		{
			vTaskDelay(1);
		}
	}
}
