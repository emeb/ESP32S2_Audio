/*
 * esp_gfx.h - ST7735 graphics using the ESP built-in graphics driver
 * 03-05-22 E. Brombaugh
 */

#ifndef __esp_gfx__
#define __esp_gfx__

esp_err_t esp_gfx_init(void);
void esp_gfx_update(int16_t *v);


#endif
