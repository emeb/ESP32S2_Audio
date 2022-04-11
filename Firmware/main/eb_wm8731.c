/*
 * eb_wm8731.c - my high-level wm8731 driver.
 * 01-18-22 E. Brombaugh
 */
#include <stdio.h>
#include "main.h"
#include "driver/i2c.h"
#include "eb_wm8731.h"

static const char *TAG = "eb_wm8731";

#define I2C_MASTER_SCL_IO           6		/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO			5		/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM				0		/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ			100000	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE	0		/*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE	0		/*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS	1000

/* The 7 bits Codec address (sent through I2C interface) */
#define W8731_ADDR_0 0x1A

#define W8731_NUM_REGS 10

const uint16_t w8731_init_data[W8731_NUM_REGS] =
{
	0x017,			// Reg 00: Left Line In (0dB, mute off)
	0x017,			// Reg 01: Right Line In (0dB, mute off)
	0x079,			// Reg 02: Left Headphone out (0dB)
	0x079,			// Reg 03: Right Headphone out (0dB)
	0x012,			// Reg 04: Analog Audio Path Control (DAC sel, Mute Mic)
	0x008,			// Reg 05: Digital Audio Path Control (mute)
	0x060,			// Reg 06: Power Down Control (Clkout, Osc, Mic Off)
	0x002,			// Reg 07: Digital Audio Interface Format (msb, 16-bit, slave, I2S)
	0x000,			// Reg 08: Sampling Control (Normal, 256x, 48k ADC/DAC)
	0x001			// Reg 09: Active Control
};

enum wm8731_regs
{
	REG_LLIN,
	REG_RLIN,
	REG_LHP,
	REG_RHP,
	REG_APATH,
	REG_DPATH,
	REG_PCTL,
	REG_DAIF,
	REG_SMPL,
	REG_ACT,
	REG_RST = 0x0f
};

static uint16_t w8731_shadow[W8731_NUM_REGS];

/*
 * top-level codec setup
 */
esp_err_t eb_wm8731_Init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*
 * reset I2C bus
 */
void eb_wm8731_i2c_reset(void)
{
	/* doesn't do anything ATM */
}

/*
 * codec write access
 */
esp_err_t eb_wm8731_write(uint8_t addr, uint16_t data)
{
    esp_err_t status = ESP_OK;
    uint8_t i2c_msg[2];

	/* update shadow regs */
	if(addr<W8731_NUM_REGS)
		w8731_shadow[addr] = data;

	/* Assemble 2-byte data in WM8731 format */
    i2c_msg[0] = ((addr<<1)&0xFE) | ((data>>8)&0x01);
	i2c_msg[1] = data&0xFF;

	/* send it */
    status = i2c_master_write_to_device(I2C_MASTER_NUM, W8731_ADDR_0, i2c_msg, sizeof(i2c_msg), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

	/* Check the communication status */
	if(status != ESP_OK)
	{
		/* Reset the I2C communication bus */
		eb_wm8731_i2c_reset();

		ESP_LOGI(TAG, "eb_wm8731_write: write to DevAddr 0x%02X / RegisterAddr 0x%02X failed - resetting.",
			W8731_ADDR_0, addr);
	}
	//else
	//	ESP_LOGI(TAG, "eb_wm8731_write: write to DevAddr 0x%02X / RegisterAddr 0x%02X = 0x%03X",
	//		W8731_ADDR_0, addr, data);

	return status;
}

/*
 * codec reset
 */
int32_t eb_wm8731_Reset(void)
{
	int32_t result = 0;
	uint8_t i;

    /* initial reset */
    eb_wm8731_write(REG_RST, 0);

	/* Load default values */
	for(i=0;i<W8731_NUM_REGS;i++)
	{
		if(eb_wm8731_write(i, w8731_init_data[i]) != ESP_OK)
            result++;
	}

	return result;
}

/*
 * mute/unmute the codec outputs
 * CAUTION - although the datasheet says this is supposed to be "soft", it
 * seems to cut off hard and cause a pop. Better to do soft muting in the
 * local DSP.
 */
void eb_wm8731_Mute(uint8_t enable)
{
	uint8_t mute = enable ? 0x08 : 0x00;

	/* send mute cmd */
	eb_wm8731_write(REG_DPATH, mute);

	/* wait a bit for it to complete */
	vTaskDelay(20/portTICK_RATE_MS);
}

/*
 * set codec headphone volume
 */
void eb_wm8731_HPVol(uint8_t vol)
{
	/* set both chls volume */
	eb_wm8731_write(REG_LHP, 0x180 | (vol & 0x7f));
}

/*
 * set codec Input Source
 */
void eb_wm8731_InSrc(uint8_t src)
{
	uint16_t temp_reg;

	/* set line or mic input */
	temp_reg = (w8731_shadow[REG_APATH] & 0x1F9) | (src ? 0x04 : 0x02);
	eb_wm8731_write(REG_APATH, temp_reg);

	/* set line/mic power */
	//temp_reg = (w8731_shadow[REG_PCTL] & 0x1FC) | (src ? 0x01 : 0x02);
	//Codec_WriteRegister(((W8731_ADDR_0)<<1), REG_PCTL, temp_reg);
}

/*
 * set codec Input volume
 */
void eb_wm8731_InVol(uint8_t vol)
{
	/* set both chls volume */
	eb_wm8731_write(REG_LLIN, 0x100 | (vol & 0x3f));
}

/*
 * set codec Mic Boost
 */
void eb_wm8731_MicBoost(uint8_t boost)
{
	uint16_t temp_reg;

	/* set/clear mic boost bit */
	temp_reg = (w8731_shadow[REG_APATH] & 0x1FE) | (boost ? 1 : 0);
	eb_wm8731_write(REG_APATH, temp_reg);
}