/*
 * eb_wm8731.h - my high-level wm8731 driver.
 * 01-18-22 E. Brombaugh
 */

#ifndef __eb_wm8731__
#define __eb_wm8731__

extern volatile uint32_t wm8731_stat;

esp_err_t eb_wm8731_Init(void);
int32_t eb_wm8731_Reset(void);
void eb_wm8731_Mute(uint8_t enable);
void eb_wm8731_HPVol(uint8_t vol);
void eb_wm8731_InSrc(uint8_t src);
void eb_wm8731_InVol(uint8_t vol);
void eb_wm8731_MicBoost(uint8_t boost);

#endif
