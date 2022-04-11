/*
 * esp32s2_audio main.c
 * 01-23-22 E. Brombaugh
 */

#include <stdio.h>
#include "main.h"
#include "esp_timer.h"
#include "eb_adc.h"
#include "audio.h"
#include "eb_wm8731.h"
#include "button.h"
#include "st7735.h"
#include "gfx.h"
#include "menu.h"
#include "dsp_lib.h"
#include "splash.h"

static const char* TAG = "esp32_audio";

/* build version in simple format */
const char *fwVersionStr = "V0.1";

/* build time */
const char *bdate = __DATE__;
const char *btime = __TIME__;

/*
 * entry point
 */
void app_main(void)
{
	printf("\n\nESP32S2_Audio - Audio DSP module %s starting\n\r", fwVersionStr);
	printf("Build Date: %s\n\r", bdate);
	printf("Build Time: %s\n\r", btime);
	printf("\n");

	/* init ADC */
    ESP_LOGI(TAG, "Init ADC");
	eb_adc_init();
	
	/* init I2S */
    ESP_LOGI(TAG, "Init Audio");
	audio_init();
	
	/* init codec */
    ESP_LOGI(TAG, "Init WM8731 Codec");
	eb_wm8731_Init();
	eb_wm8731_Reset();
	eb_wm8731_Mute(0);
	
	/* init button */
    ESP_LOGI(TAG, "Init button");
	button_init();
	
	/* Init LCD */
	ESP_LOGI(TAG, "Init GFX");
	gfx_init(&ST7735_drvr);
	gfx_bitblt(0, 0, 160, 80, (uint16_t *)splash_png_565);
	gfx_set_forecolor(GFX_BLACK);
	gfx_set_backcolor(0xc7def6);
	gfx_drawstrctr(120, 20, "ESP32S2");
	gfx_drawstrctr(120, 30, "Audio");
	gfx_drawstrctr(120, 40, (char *)fwVersionStr);
	gfx_set_forecolor(GFX_WHITE);
	gfx_set_backcolor(GFX_BLACK);
	ST7735_setBacklight(1);
	vTaskDelay(3000/portTICK_RATE_MS);
		
	/* init menu */
    ESP_LOGI(TAG, "Init menu");
	menu_init();
	
	/* loop here forever */
    ESP_LOGI(TAG, "Looping...");
    while(1)
	{		
		menu_update();
		//printf("%5d %5d\n", adc_val[0], adc_val[1]);
		vTaskDelay(1);
    }
}
