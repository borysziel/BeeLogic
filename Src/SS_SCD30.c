/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensirion_arch_config.h"
#include "sensirion_i2c.h"
#include "sensirion_common.h"
#include "scd_git_version.h"
#include "SS_SCD30.h"
#include "gpio.h"


extern I2C_HandleTypeDef hi2c1;
static const u8 SCD_I2C_ADDRESS = 0x61;


#define SCD_CMD_START_PERIODIC_MEASUREMENT  0x0010
#define SCD_CMD_STOP_PERIODIC_MEASUREMENT   0x0104
#define SCD_CMD_READ_MEASUREMENT            0x0300
#define SCD_CMD_SET_MEASUREMENT_INTERVAL    0x4600
#define SCD_CMD_GET_DATA_READY              0x0202
#define SCD_CMD_SET_TEMPERATURE_OFFSET      0x5403
#define SCD_CMD_SET_ALTITUDE                0x5102
#define SCD_CMD_SET_FORCED_RECALIBRATION    0x5204
#define SCD_CMD_AUTO_SELF_CALIBRATION       0x5306

#define SCD_WORD_LEN     2
#define SCD_COMMAND_LEN  2
#define SCD_MAX_BUFFER_WORDS 24
#define SCD_CMD_SINGLE_WORD_BUF_LEN (SCD_COMMAND_LEN + SCD_WORD_LEN + CRC8_LEN)


/**
 * scd_i2c_read_words() - read data words from sensor
 *
 * @data:       Allocated buffer to store the read data.
 *              The buffer may also have been modified on STATUS_FAIL return.
 * @data_words: Number of data words to read (without CRC bytes)
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
//void SCD30_I2C_Config(void)
//{
//		GPIO_InitTypeDef     GPIO_InitStruct;
//	    I2C_InitTypeDef     I2C_InitStruct;
//
//	    __HAL_RCC_I2C1_CLK_ENABLE();
//	    __HAL_RCC_GPIOB_CLK_ENABLE();
//	    /*
//	     * PB8 -> I2C1_SCL
//	     * PB9 -> I2C2_SDA
//	     */
//	    GPIO_InitStruct.Pin         = SENSOR_D_SCL_Pin|SENSOR_D_SDA_Pin;
//	    GPIO_InitStruct.Mode         = GPIO_MODE_AF_OD;
//	    GPIO_InitStruct.Speed         = GPIO_SPEED_FREQ_VERY_HIGH;
//	    GPIO_InitStruct.Pull         = GPIO_PULLUP;
//	    GPIO_InitStruct.Alternate     = 0x04; //alternate function mapping
//	    HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
//
//	    /*
//	     * HAL_NVIC conf
//	     */
//	    HAL_NVIC_SetPriority(I2C1_EV_IRQn,0,0);
//	    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
//	    HAL_NVIC_SetPriority(I2C1_ER_IRQn,0,0);
//	    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
//
//	    hi2c3.Instance = I2C1;
//	    hi2c3.Init.ClockSpeed = 100000;
//	    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
//	    hi2c3.Init.OwnAddress1 = 0;
//	    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
//	    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
//	    hi2c3.Init.OwnAddress2 = 0;
//	    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
//	    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
//	    if(HAL_I2C_Init(&hi2c1) != HAL_OK)
//	        while(1);
//
//
//
//}
static s16 scd_i2c_read_words(u16 *data, u16 data_words) {
    s16 ret;
    u16 i, j;
    u16 size = data_words * (SCD_WORD_LEN + CRC8_LEN);
    u16 word_buf[SCD_MAX_BUFFER_WORDS];
    u8 * const buf8 = (u8 *)word_buf;

    ret = sensirion_i2c_read(SCD_I2C_ADDRESS, buf8, size);
    if (ret != STATUS_OK)
        return ret;

    /* check the CRC for each word */
    for (i = 0, j = 0;
         i < size;
         i += SCD_WORD_LEN + CRC8_LEN, j += SCD_WORD_LEN) {

        if (sensirion_common_check_crc(&buf8[i], SCD_WORD_LEN,
                                       buf8[i + SCD_WORD_LEN]) == STATUS_FAIL) {
            return STATUS_FAIL;
        }
        ((u8 *)data)[j]     = buf8[i];
        ((u8 *)data)[j + 1] = buf8[i + 1];
    }

    return STATUS_OK;
}


/**
 * scd_fill_cmd_send_buf() - create the i2c send buffer for a command and a set
 *                           of argument words.
 *                           The output buffer interleaves argument words with
 *                           their checksums.
 * @buf:        The generated buffer to send over i2c. Then buffer length must
 *              be at least SCD_COMMAND_LEN + num_args * (SCD_WORD_LEN +
 *              CRC8_LEN).
 * @cmd:        The i2c command to send. It already includes a checksum.
 * @args:       The arguments to the command. Can be NULL if none.
 * @num_args:   The number of word arguments in args.
 */
static void scd_fill_cmd_send_buf(u8 *buf, u16 cmd, const u16 *args,
                                  u8 num_args) {
    u8 crc;
    u8 i;
    u8 idx = 0;

    buf[idx++] = (u8)((cmd & 0xFF00) >> 8);
    buf[idx++] = (u8)((cmd & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i) {
        crc = sensirion_common_generate_crc((u8 *)&args[i], SCD_WORD_LEN);

        buf[idx++] = (u8)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (u8)((args[i] & 0x00FF) >> 0);
        buf[idx++] = crc;
    }
}


/**
 * scd_i2c_write() - writes to the sensor
 * @command:    Sensor command
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
static s16 scd_i2c_write(u16 command) {
    u8 buf[SCD_COMMAND_LEN];

    scd_fill_cmd_send_buf(buf, command, NULL, 0);
    return sensirion_i2c_write(SCD_I2C_ADDRESS, buf, SCD_COMMAND_LEN);
}


/**
 * scd_i2c_read_words_from_cmd() - reads data words from the sensor after a
 *                                 command has been issued
 * @cmd:        Command
 * @data_words: Allocated buffer to store the read data
 * @num_words:  Data words to read (without CRC bytes)
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
static s16 scd_i2c_read_words_from_cmd(u16 cmd, u16 *data_words,
                                       u16 num_words) {

    s16 ret = scd_i2c_write(cmd);

    if (ret != STATUS_OK)
        return ret;

    return scd_i2c_read_words(data_words, num_words);
}


s16 scd_start_periodic_measurement(u16 ambient_pressure_mbar) {
    u8 buf[SCD_CMD_SINGLE_WORD_BUF_LEN];

    if (ambient_pressure_mbar && (ambient_pressure_mbar < 700 ||
                                  ambient_pressure_mbar > 1400)) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    scd_fill_cmd_send_buf(buf, SCD_CMD_START_PERIODIC_MEASUREMENT,
                          &ambient_pressure_mbar,
                          sizeof(ambient_pressure_mbar) / SCD_WORD_LEN);

    return sensirion_i2c_write(SCD_I2C_ADDRESS, buf, sizeof(buf));
}


s16 scd_stop_periodic_measurement() {
    return scd_i2c_write(SCD_CMD_STOP_PERIODIC_MEASUREMENT);
}


s16 scd_read_measurement(f32 *co2_ppm, f32 *temperature, f32 *humidity) {
    s16 ret;
    u32 word_buf[3]; /* 32bit or 2 words for each co2, temperature, humidity */
    union {
        u32 raw;
        f32 float32;
    } tmp;

    ret = scd_i2c_read_words_from_cmd(SCD_CMD_READ_MEASUREMENT, (u16 *)word_buf,
                                      sizeof(word_buf) / SCD_WORD_LEN);
    if (ret != STATUS_OK)
        return ret;

    tmp.raw = be32_to_cpu(word_buf[0]);
    *co2_ppm = tmp.float32;

    tmp.raw = be32_to_cpu(word_buf[1]);
    *temperature = tmp.float32;

    tmp.raw = be32_to_cpu(word_buf[2]);
    *humidity = tmp.float32;

    return STATUS_OK;
}


s16 scd_set_measurement_interval(u16 interval_sec) {
    u8 buf[SCD_CMD_SINGLE_WORD_BUF_LEN];

    if (interval_sec < 2 || interval_sec > 1800) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    scd_fill_cmd_send_buf(buf, SCD_CMD_SET_MEASUREMENT_INTERVAL, &interval_sec,
                          sizeof(interval_sec) / SCD_WORD_LEN);

    return sensirion_i2c_write(SCD_I2C_ADDRESS, buf, sizeof(buf));
}


s16 scd_get_data_ready(u16 *data_ready) {
    return scd_i2c_read_words_from_cmd(SCD_CMD_GET_DATA_READY, data_ready,
                                       sizeof(*data_ready) / SCD_WORD_LEN);
}


s16 scd_set_temperature_offset(u16 temperature_offset) {
    u8 buf[SCD_CMD_SINGLE_WORD_BUF_LEN];

    scd_fill_cmd_send_buf(buf, SCD_CMD_SET_TEMPERATURE_OFFSET,
                          &temperature_offset,
                          sizeof(temperature_offset) / SCD_WORD_LEN);

    return sensirion_i2c_write(SCD_I2C_ADDRESS, buf, sizeof(buf));
}


s16 scd_set_altitude(u16 altitude) {
    u8 buf[SCD_CMD_SINGLE_WORD_BUF_LEN];

    scd_fill_cmd_send_buf(buf, SCD_CMD_SET_ALTITUDE, &altitude,
                          sizeof(altitude) / SCD_WORD_LEN);

    return sensirion_i2c_write(SCD_I2C_ADDRESS, buf, sizeof(buf));
}


s16 scd_get_automatic_self_calibration(u8 *asc_enabled) {
    u16 word;
    s16 ret;

    ret = scd_i2c_read_words_from_cmd(SCD_CMD_AUTO_SELF_CALIBRATION,
                                      &word, sizeof(word) / SCD_WORD_LEN);
    if (ret == 0)
        *asc_enabled = (u8)word;

    return ret;
}


s16 scd_enable_automatic_self_calibration(u8 enable_asc) {
    u8 buf[SCD_CMD_SINGLE_WORD_BUF_LEN];
    u16 asc = !!enable_asc;

    scd_fill_cmd_send_buf(buf, SCD_CMD_AUTO_SELF_CALIBRATION, &asc,
                          sizeof(asc) / SCD_WORD_LEN);

    return sensirion_i2c_write(SCD_I2C_ADDRESS, buf, sizeof(buf));
}


s16 scd_set_forced_recalibration(u16 co2_ppm) {
    u8 buf[SCD_CMD_SINGLE_WORD_BUF_LEN];

    scd_fill_cmd_send_buf(buf, SCD_CMD_SET_FORCED_RECALIBRATION, &co2_ppm,
                          sizeof(co2_ppm) / SCD_WORD_LEN);

    return sensirion_i2c_write(SCD_I2C_ADDRESS, buf, sizeof(buf));
}


const char *scd_get_driver_version() {
    return SCD_DRV_VERSION_STR;
}


u8 scd_get_configured_address() {
    return SCD_I2C_ADDRESS;
}


s16 scd_probe() {
    u16 data_ready;

    /* Initialize I2C */
    sensirion_i2c_init();

    /* try to read data-ready state */
    return scd_get_data_ready(&data_ready);
}
