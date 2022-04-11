/*
 * fx.c - Algorithm access for esp32s2_audio
 * 03-13-22 E. Brombaugh
 */
 
#include <stdio.h>
#include <string.h>
#include "fx.h"
#include "fx_vca.h"
#include "fx_cdl.h"

static const char* TAG = "fx";

/* external memory buffer */
int16_t *fx_ext_buffer;
size_t fx_ext_sz;

/* 32kB pre-allocated internal memory for DSP */
size_t fx_int_sz;
uint32_t *fx_mem;

/* pointer to the fx data structure is void and recast inside the fns */
void *fx;

/* currently active algo */
uint8_t fx_algo;


/**************************************************************************/
/******************* Bypass algo definition *******************************/
/**************************************************************************/

const char *bypass_param_names[] =
{
	"",
	"",
	"",
};

/*
 * Bypass init
 */
void * fx_bypass_Init(uint32_t *mem)
{
	/* no init - just return pointer */
	return (void *)mem;
}

/*
 * Bypass Cleanup
 */
void fx_bypass_Cleanup(void *dummy)
{
	/* does nothing */
}

/*
 * Bypass audio process is just in-out loopback / bypass
 */
void fx_bypass_Proc(void *dummy, int16_t *dst, int16_t *src, uint16_t sz)
{
	while(sz--)
	{
		*dst++ = *src++;
		*dst++ = *src++;
	}
}

/*
 * Bypass render parameter is just simple percentage
 */
void fx_bypass_Render_Parm(void *vblk, uint8_t idx)
{
	char txtbuf[32];
	GFX_RECT rect =
	{
		.x0 = 65,
		.y0 = idx*10+10,
		.x1 = 158,
		.y1 = idx*10+17
	};
	
	if(idx == 0)
		return;

	sprintf(txtbuf, "%2d%% ", adc_param[idx]/41);
	gfx_drawstrrect(&rect, txtbuf);
}

/*
 * Bypass init
 */
const fx_struct fx_bypass_struct =
{
	"Bypass",
	0,
	bypass_param_names,
	fx_bypass_Init,
	fx_bypass_Cleanup,
	fx_bypass_Proc,
	fx_bypass_Render_Parm,
};


/**************************************************************************/

/* array of effect structures */
const fx_struct *effects[FX_NUM_ALGOS] = {
	&fx_bypass_struct,
	&fx_vca_struct,
	&fx_cdr_struct,
};

/*
 * initialize the effects library
 */
void fx_init(void)
{
	/* allocate ~2MB external buffer memory */
	fx_int_sz = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
	ESP_LOGI(TAG, "%d bytes internal available for audio", fx_int_sz);
	fx_mem = heap_caps_malloc(FX_MAX_MEM, MALLOC_CAP_SPIRAM);
	if(fx_mem)
		ESP_LOGI(TAG, "%d bytes internal reserved for audio", FX_MAX_MEM);
	else
		ESP_LOGI(TAG, "Failed getting %d bytes internal for audio", FX_MAX_MEM);
	
	/* allocate ~2MB external buffer memory */
	fx_ext_sz = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
	fx_ext_buffer = heap_caps_malloc(fx_ext_sz, MALLOC_CAP_SPIRAM);
	if(fx_ext_buffer)
		ESP_LOGI(TAG, "%d bytes PSRAM available for audio buffers", fx_ext_sz);
	else
		ESP_LOGI(TAG, "Failed getting %d bytes PSRAM for audio buffers", fx_ext_sz);

	/* start off with bypass algo */
	fx_algo = 0;
	fx = effects[fx_algo]->init(fx_mem);
}

/*
 * switch algorithms
 */
void fx_select_algo(uint8_t algo)
{
	uint8_t prev_fx_algo;
	
	/* only legal algorithms */
	if(algo >= FX_NUM_ALGOS)
		return;
	
	/* bypass during init */
	prev_fx_algo = fx_algo;
	fx_algo = 0;
	
	/* cleanup previous effect */
	effects[prev_fx_algo]->cleanup(fx);
	
	/* init next effect from effect array */
	fx = effects[algo]->init(fx_mem);
		
	/* switch to next effect */
	fx_algo = algo;
}

/*
 * process audio through current effect
 */
void IRAM_ATTR fx_proc(int16_t *dst, int16_t *src, uint16_t sz)
{
	/* use effect structure function pointers */
	effects[fx_algo]->proc(fx, dst, src, sz);
}

/*
 * get current algorithm
 */
uint8_t fx_get_algo(void)
{
	return fx_algo;
}

/*
 * get number of params
 */
uint8_t fx_get_num_parms(void)
{
	return effects[fx_algo]->parms;
}

/*
 * get name of effect
 */
char * fx_get_algo_name(void)
{
	return (char *)effects[fx_algo]->name;
}

/*
 * get name of param by index
 */
char * fx_get_parm_name(uint8_t idx)
{
	return (char *)effects[fx_algo]->parm_names[idx];
}

/*
 * render parameter parts
 */
void fx_render_parm(uint8_t idx)
{
	effects[fx_algo]->render_parm(fx, idx);
}



