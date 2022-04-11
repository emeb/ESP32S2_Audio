/*
 * menu.c - menu / UI handler for ESP32S2 Audio
 * 03-07-22 E. Brombaugh
 */
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "button.h"
#include "gfx.h"
#include "widgets.h"
#include "audio.h"
#include "menu.h"
#include "fx.h"
#include "eb_wm8731.h"

#define MENU_INTERVAL 50000
#define MENU_MAX_PARAMS (FX_MAX_PARAMS+1)
#define MENU_VU_WIDTH 50

enum save_flags
{
	SAVE_ACT = 1,
	SAVE_VALUE = 2,
	SAVE_ALGO = 4,
};

static const char* TAG = "menu";
static int16_t menu_item_values[FX_NUM_ALGOS][MENU_MAX_PARAMS];
static uint8_t menu_reset, menu_act_item, menu_save_mask;
static uint8_t menu_value_scoreboard[FX_NUM_ALGOS];
static uint16_t menu_algo, menu_save_counter;
static uint64_t menu_time;
static char txtbuf[32];

/*
 * load values from NVS
 */
esp_err_t menu_load_state(void)
{
	uint8_t i,j, commit=0;
	
    /* Initialize NVS */
    esp_err_t err = nvs_flash_init();
    if(err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
        // NVS partition was truncated and needs to be erased
		ESP_LOGW(TAG, "menu_load_values: Erasing NVS");
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

	/* Open NVS */
    //ESP_LOGI(TAG, "menu_load_values: Opening NVS");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if(err != ESP_OK)
	{
        ESP_LOGW(TAG, "menu_load_values: Error (%s) opening NVS", esp_err_to_name(err));
    }
	else
	{
		/* get algo */
		menu_algo = 0;
		err = nvs_get_u16(my_handle, "menu_algo", &menu_algo);
		//ESP_LOGI(TAG, "menu_algo = %d, err = %s", menu_algo, esp_err_to_name(err));
		if(err == ESP_ERR_NVS_NOT_FOUND)
		{
			ESP_LOGI(TAG, "created menu_algo = %d, err = %s", menu_algo, esp_err_to_name(err));
			err = nvs_set_u16(my_handle, "menu_algo", menu_algo);
			commit = 1;
		}
		
		/* get active item */
		menu_act_item = 0;
		err = nvs_get_u8(my_handle, "menu_act_item", &menu_act_item);
		//ESP_LOGI(TAG, "menu_act_item = %d, err = %s", menu_act_item, esp_err_to_name(err));
		if(err == ESP_ERR_NVS_NOT_FOUND)
		{
			err = nvs_set_u8(my_handle, "menu_act_item", menu_act_item);
			ESP_LOGI(TAG, "created menu_act_item = %d, err = %s", menu_act_item, esp_err_to_name(err));
			commit = 1;
		}
		
		/* get params */
		for(i=0;i<FX_NUM_ALGOS;i++)
		{
			for(j=0;j<MENU_MAX_PARAMS;j++)
			{
				int16_t raw_param = 0;
				sprintf(txtbuf, "pvalues_%2d_%2d", i, j);
				err = nvs_get_i16(my_handle, txtbuf, &raw_param);
				//ESP_LOGI(TAG, "get: %s = %d, err = %s", txtbuf, raw_param, esp_err_to_name(err));
				if(err == ESP_ERR_NVS_NOT_FOUND)
				{
					err = nvs_set_i16(my_handle, txtbuf, raw_param);
					ESP_LOGI(TAG, "created: %s = %d, err = %s", txtbuf, raw_param, esp_err_to_name(err));
					commit = 1;
				}
				menu_item_values[i][j] = raw_param;
			}
		}
		
		if(commit)
		{
			err = nvs_commit(my_handle);
			ESP_LOGI(TAG, "commit: err = %s", esp_err_to_name(err));
		} 
		
        nvs_close(my_handle);
	}

	return ESP_OK;
}

/*
 * write updated state back to NVS
 */
void menu_save_state(void)
{
	esp_err_t err;

	/* Open NVS */
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if(err != ESP_OK)
	{
        ESP_LOGW(TAG, "menu_save_value: Error (%s) opening NVS", esp_err_to_name(err));
    }
	else
	{
		//ESP_LOGI(TAG, "menu_save_value: Success Opening NVS");
		
		/* algo */
		if(menu_save_mask & SAVE_ALGO)
		{
			err = nvs_set_u16(my_handle, "menu_algo", menu_algo);
			//ESP_LOGI(TAG, "set menu_algo = %d, err = %s", menu_algo, esp_err_to_name(err));
		}
		
		/* active param */
		if(menu_save_mask & SAVE_ACT)
		{
			err = nvs_set_u8(my_handle, "menu_act_item", menu_act_item);
			//ESP_LOGI(TAG, "set menu_act_item = %d, err = %s", menu_act_item, esp_err_to_name(err));
		}
		
		/* values */
		if(menu_save_mask & SAVE_ACT)
		{
			/* loop over scoreboard and set all params that have been marked */
			for(int i=0;i<FX_NUM_ALGOS;i++)
			{
				for(int j=0;j<MENU_MAX_PARAMS;j++)
				{
					if(menu_value_scoreboard[i] & (1<<j))
					{
						sprintf(txtbuf, "pvalues_%2d_%2d", i, j);
						err = nvs_set_i16(my_handle, txtbuf, menu_item_values[i][j]);
						//ESP_LOGI(TAG, "set %s = %d, err = %s", txtbuf, menu_item_values[i][j], esp_err_to_name(err));
					}
				}
				menu_value_scoreboard[i] = 0;
			}
		}
		
		menu_save_mask = 0;
		
        // Commit written value.
        err = nvs_commit(my_handle);
        //ESP_LOGI(TAG, "commit: %s", esp_err_to_name(err));
		
        nvs_close(my_handle);
	}
}

/*
 * periodic menu updates
 */
void menu_timer_callback(void)
{
	/* update load */
	gfx_set_forecolor(GFX_WHITE);
	uint64_t period = audio_load[0] - audio_load[2];
	uint64_t duration = audio_load[1] - audio_load[0];
	uint64_t load;
	
	/* update load indicator */
	if(period != 0)
	{
		load = 100*duration/period;
		sprintf(txtbuf, "%2d%% ", (uint32_t)load);
		gfx_drawstr(40, 0, txtbuf);
	}
	
	/* update state save */
	if(menu_save_counter != 0)
	{
		menu_save_counter--;
		
		if(menu_save_counter == 0)
		{
			ESP_LOGI(TAG, "menu_timer_callback: Saving State");
			menu_save_state();
			gfx_set_forecolor(GFX_GREEN);
			gfx_fillcircle(156, 3, 3);
			gfx_set_forecolor(GFX_WHITE);
		}
	}

	menu_item_values[menu_algo][menu_act_item] = adc_param[menu_act_item];
	fx_render_parm(menu_act_item);
	
	/* update mix */
	widg_sliderH(30, 50, 100, 8, adc_val[0]/41);
	
	/* update VU meters */
	widg_bargraphHG(20, 60, MENU_VU_WIDTH, 8, audio_sl[1]/328); audio_sl[1] = 0;
	widg_bargraphHG(20, 70, MENU_VU_WIDTH, 8, audio_sl[0]/328); audio_sl[0] = 0;
	widg_bargraphHG(100, 60, MENU_VU_WIDTH, 8, audio_sl[3]/328); audio_sl[3] = 0;
	widg_bargraphHG(100, 70, MENU_VU_WIDTH, 8, audio_sl[2]/328); audio_sl[2] = 0;
}

/*
 * schedule save state in the future
 */
void menu_sched_save(uint8_t mask)
{
	ESP_LOGI(TAG, "menu_sched_save: Scheduling save, mask = 0x%02X", mask);
	menu_save_mask |= mask;
	if(mask & SAVE_VALUE)
		menu_value_scoreboard[menu_algo] |= 1<<menu_act_item;
	menu_save_counter = 5000000/MENU_INTERVAL;	 // 5 sec
	gfx_set_forecolor(GFX_RED);
	gfx_fillcircle(156, 3, 3);
	gfx_set_forecolor(GFX_WHITE);
}

/*
 * render any changes in the menu
 */
void menu_render(void)
{
	uint8_t i;
	char *name;
	GFX_RECT rect;
	
	/* set constants */
	gfx_set_forecolor(GFX_WHITE);
	GFX_COLOR fgcolor = gfx_get_forecolor(), bgcolor = gfx_get_backcolor();
	
	/* update active item */
	for(i=0;i<MENU_MAX_PARAMS;i++)
	{
		/* highlight active item */
		if(menu_act_item == i)
			gfx_set_forecolor(GFX_MAGENTA);
		else
			gfx_set_forecolor(bgcolor);
		rect.x0 = 0;
		rect.y0 = i*10+9;
		rect.x1 = 159;
		rect.y1 = rect.y0+9;
		gfx_drawrect(&rect);
		
		/* item names */
		gfx_set_forecolor(fgcolor);
		if(i == 0)
		{
			name = "Algo:";
			gfx_drawstr(1, i*10+10, name);
		}
		else
		{
			if(i<fx_get_num_parms()+1)
			{
				/* render parm name and clear rest of line */
				//fx_render_parm(i, PARM_NAME);
				gfx_drawstr(1, i*10+10, fx_get_parm_name(i-1));
			}
			else
			{
				/* past last parm so clear whole line */
				rect.x0 = 1;
				rect.y0 = i*10+10;
				rect.x1 = 158;
				rect.y1 = rect.y0+7;
				gfx_clrrect(&rect);
			}
		}
		
		/* init all item values */
		if(menu_reset)
		{
			if(i == 0)
			{
				sprintf(txtbuf, "%s             ", fx_get_algo_name());
				txtbuf[13] = 0;	// max 13 chars 
				gfx_drawstr(48, i*10+10, txtbuf);
			}
			else
			{
				if(i<fx_get_num_parms()+1)
				{
					fx_render_parm(i);
				}
			}
		}
	}
	
	/* refresh static items */
	if(menu_reset)
	{
		menu_reset = 0;
		
		/* save state */
		if(menu_save_counter)
			gfx_set_forecolor(GFX_RED);
		else
			gfx_set_forecolor(GFX_GREEN);
		gfx_fillcircle(156, 3, 3);
		
		/* load % */
		gfx_set_forecolor(GFX_WHITE);
		gfx_drawstr(0, 0, "Load");
		
		/* W/D mix */
		gfx_drawstr(0,   50, "Dry");
		gfx_drawstr(136, 50, "Wet");
		
		/* vu meters labels and boxes */
		gfx_drawstr(0, 61, "il");
		widg_bargraphH(20, 60, MENU_VU_WIDTH, 8, 0);
		gfx_drawstr(0, 71, "ir");
		widg_bargraphH(20, 70, MENU_VU_WIDTH, 8, 0);
		gfx_drawstr(80, 61, "ol");
		widg_bargraphH(100, 60, MENU_VU_WIDTH, 8, 0);
		gfx_drawstr(80, 71, "or");
		widg_bargraphH(100, 70, MENU_VU_WIDTH, 8, 0);
		
	}
}

/*
 * initialize menu handler
 */
void menu_init(void)
{
	uint8_t i;
	
 	/* init menu state */
	ESP_LOGI(TAG, "menu_init: zeroing");
	menu_reset = 1;
	menu_save_mask = 0;
	menu_save_counter = 0;
	for(i=0;i<FX_NUM_ALGOS;i++)
		menu_value_scoreboard[i] = 0;
	
	/* load stored state */
	menu_load_state();
	eb_adc_setactparam(menu_act_item);
	for(i=0;i<MENU_MAX_PARAMS;i++)
		eb_adc_setparamval(i, menu_item_values[menu_algo][i]);
	eb_adc_forceactparam();
	fx_select_algo(menu_algo);
	audio_mute(0);	// initial unmute after algo selected
	ESP_LOGI(TAG, "menu_init: state loaded act=%d, algo=%d", menu_act_item, menu_algo);
	
	/* init VU gradient */
	widg_gradient_init(MENU_VU_WIDTH);
	
	/* initial draw of menu */
	gfx_clrscreen();
	menu_render();
	
	/* set time for next update */
	menu_time = esp_timer_get_time() + MENU_INTERVAL;
}

/*
 * periodic menu update call
 */
void menu_update(void)
{
	/* look for button press */
	if(button_re())
	{
		menu_sched_save(SAVE_ACT | SAVE_VALUE);
		menu_act_item++;
		menu_act_item = menu_act_item < fx_get_num_parms()+1 ? menu_act_item : 0;
		eb_adc_setactparam(menu_act_item);
		ESP_LOGI(TAG, "menu_update: act_item = %d", menu_act_item);
		menu_render();
	}
	
	/* test for algo change */
	if(menu_act_item == 0)
	{
		if(dsp_ratio_hyst_arb(&menu_algo, adc_param[0], FX_NUM_ALGOS-1))
		{
			ESP_LOGI(TAG, "menu_update: algo = %d", menu_algo);
			audio_mute(1);
			menu_sched_save(SAVE_ALGO);
			fx_select_algo(menu_algo);
			for(int i=1;i<MENU_MAX_PARAMS;i++)
				eb_adc_setparamval(i, menu_item_values[menu_algo][i]);
			menu_reset = 1;
			menu_render();
			audio_mute(0);
		}
	}
	
	/* periodic updates in foreground to avoid conflicts */
	if(esp_timer_get_time() >= menu_time)
	{
		menu_time = esp_timer_get_time() + MENU_INTERVAL;
		menu_timer_callback();
	}
}
