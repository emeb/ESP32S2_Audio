/*
 * button.h - handle the button
 * 03-06-22 E. Brombaugh
 */

#ifndef __button__
#define __button__

esp_err_t button_init(void);
uint8_t button_get(void);
uint8_t button_fe(void);
uint8_t button_re(void);

#endif
