/*
 * button.c - handle the button
 * 03-06-22 E. Brombaugh
 */

#include <stdio.h>
#include "main.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "debounce.h"
#include "button.h"

#define BUTTON_PIN 0

debounce_state btn_dbs;
uint8_t btn_fe, btn_re;

/*
 * button scanning callback
 */
void button_timer_callback(void *arg)
{
	debounce(&btn_dbs, (!gpio_get_level(BUTTON_PIN)));
	btn_fe |= btn_dbs.fe;
	btn_re |= btn_dbs.re;
}

/*
 * init button scanning
 */
esp_err_t button_init(void)
{
	/* GPIO in */
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);
	
	/* init debounce */
	init_debounce(&btn_dbs, 15);
	btn_fe = 0;
	btn_re = 0;
	
	/* start timer */
	esp_timer_handle_t timer_handle;
	esp_timer_create_args_t timer_args = {
		.callback = button_timer_callback,
		.arg = NULL,
		.name = "button_timer",
		.skip_unhandled_events = true
	};
	esp_timer_create(&timer_args, &timer_handle);
	esp_timer_start_periodic(timer_handle, 1000);
	
	return ESP_OK;
}

/*
 * check status of button
 */
uint8_t button_get(void)
{
	return btn_dbs.state;
}

/*
 * check for falling edge of button
 */
uint8_t button_fe(void)
{
	uint8_t result = btn_fe;
	btn_fe = 0;
	return result;
}

/*
 * check for rising edge of button
 */
uint8_t button_re(void)
{
	uint8_t result = btn_re;
	btn_re = 0;
	return result;
}
