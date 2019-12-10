//
// Created by rarvolt on 04.12.2019.
//

#include <esp_err.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/task.h>
#include "tsl2561.h"

#define TSL2561_I2C_PORT I2C_NUM_0


// Auto-gain thresholds
#define TSL2561_AGC_THI_13MS      (4850)    ///< Max value at Ti 13ms = 5047
#define TSL2561_AGC_TLO_13MS      (100)     ///< Min value at Ti 13ms = 100
#define TSL2561_AGC_THI_101MS     (36000)   ///< Max value at Ti 101ms = 37177
#define TSL2561_AGC_TLO_101MS     (200)     ///< Min value at Ti 101ms = 200
#define TSL2561_AGC_THI_402MS     (63000)   ///< Max value at Ti 402ms = 65535
#define TSL2561_AGC_TLO_402MS     (500)     ///< Min value at Ti 402ms = 500

// Clipping thresholds
#define TSL2561_CLIPPING_13MS     (4900)    ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_101MS    (37000)   ///< # Counts that trigger a change in gain/integration
#define TSL2561_CLIPPING_402MS    (65000)   ///< # Counts that trigger a change in gain/integration

// Delay for integration times
#define TSL2561_DELAY_INTEG_13MS    (15)    ///< Wait 15ms for 13ms integration
#define TSL2561_DELAY_INTEG_101MS   (120)   ///< Wait 120ms for 101ms integration
#define TSL2561_DELAY_INTEG_402MS   (450)   ///< Wait 450ms for 402ms integration

const char *TSL_TAG = "TSL2561";


typedef enum
{
    TSL2561_REG_CONTROL = 0x0,
    TSL2561_REG_TIMING,
    TSL2561_REG_THRESH_LL,
    TSL2561_REG_THRESH_LH,
    TSL2561_REG_THRESH_HL,
    TSL2561_REG_THRESH_HH,
    TSL2561_REG_INTERRUPT,
    TSL2561_REG_ID = 0xA,
    TSL2561_REG_DATA0_L = 0xC,
    TSL2561_REG_DATA0_H,
    TSL2561_REG_DATA1_L,
    TSL2561_REG_DATA1_H
} TSL2561_REG_t;

typedef struct
{
    uint8_t cmd : 1;
    uint8_t clear : 1;
    uint8_t word : 1;
    uint8_t block : 1;
    uint8_t address : 4;
} TSL2561_reg_command_t;

typedef struct
{
    uint8_t _reserved : 6;
    uint8_t power : 2;
} TSL2561_reg_control_t;

typedef struct
{
    uint8_t _reserved1 : 3;
    uint8_t gain : 1;
    uint8_t manual : 1;
    uint8_t _reserved2 : 1;
    uint8_t integ : 2;
} TSL2561_reg_timing_t;

typedef struct
{
    uint8_t low_byte;
    uint8_t high_byte;
} TSL2561_reg_dword_t;

typedef struct
{
    uint8_t _reserved : 2;
    uint8_t intr : 2;
    uint8_t persist : 4;
} TSL2561_reg_interrupt_t;

typedef struct
{
    uint8_t part_no : 4;
    uint8_t rev_no : 4;
} TSL2561_reg_id_t;

struct TSL2561
{
    TSL2561_ADDR_SEL_t addr;
    TSL2561_reg_id_t id;
    TSL2561_config_t config;
    bool auto_gain;
};

inline uint8_t _tsl2561_reg_command_get(TSL2561_reg_command_t *reg)
{
    return (reg->cmd << 7) |
            (reg->clear << 6) |
            (reg->word << 5) |
            (reg->block << 4) |
            reg->address;
}

inline uint8_t _tsl2561_reg_control_get(TSL2561_reg_control_t *reg)
{
    return reg->power;
}

inline uint8_t _tsl2561_reg_timing_get(TSL2561_reg_timing_t *reg)
{
    return (reg->gain << 4) |
            (reg->manual << 3) |
            reg->integ;
}

inline uint8_t _tsl2561_reg_interrupt_get(TSL2561_reg_interrupt_t *reg)
{
    return (reg->intr << 4) |
            reg->persist;
}


esp_err_t _tsl2561_reg_write8(TSL2561_handle_t tsl, uint8_t reg, uint8_t data);
esp_err_t _tsl2561_reg_write16(TSL2561_handle_t tsl, uint8_t reg, uint16_t data);
esp_err_t _tsl2561_reg_read8(TSL2561_handle_t tsl, uint8_t reg, uint8_t *data);
esp_err_t _tsl2561_reg_read16(TSL2561_handle_t tsl, uint8_t reg, uint16_t *data);
TSL2561_reg_id_t _tsl2561_read_id(TSL2561_handle_t tsl);
esp_err_t _tsl2561_write_config(TSL2561_handle_t tsl);
esp_err_t _tsl2561_get_data(TSL2561_handle_t tsl, uint16_t *broadband, uint16_t *ir);
esp_err_t _tsl2561_read_ch0(TSL2561_handle_t tsl, uint16_t *data);
esp_err_t _tsl2561_read_ch1(TSL2561_handle_t tsl, uint16_t *data);
uint32_t _tsl2561_calc_lux(uint8_t i_gain, uint8_t t_int, uint16_t ch0, uint16_t ch1);


TSL2561_handle_t tsl2561_init(TSL2561_ADDR_SEL_t addr, TSL2561_config_t *config)
{
    TSL2561_handle_t tsl = malloc(sizeof(struct TSL2561));
    tsl->addr = addr;
    tsl->id = _tsl2561_read_id(tsl);

    ESP_LOGI(TSL_TAG, "TSL2561 detected ID partno: 0x%X, revno: 0x%X",
            tsl->id.part_no, tsl->id.rev_no);

    ESP_LOGI(TSL_TAG, "Setting configuration");
    tsl2561_set_config(tsl, config);

    vTaskDelay(100 / portTICK_RATE_MS);

    return tsl;
}

void tsl2561_set_config(TSL2561_handle_t tsl, TSL2561_config_t *config)
{
    tsl->config = *config;
    _tsl2561_write_config(tsl);
}

void tsl2561_enable(TSL2561_handle_t tsl)
{
    tsl->config.power = TSL2561_PWR_ON;
    _tsl2561_write_config(tsl);
}

void tsl2561_disable(TSL2561_handle_t tsl)
{
    tsl->config.power = TSL2561_PWR_OFF;
    _tsl2561_write_config(tsl);
}

void tsl2561_enable_auto_range(TSL2561_handle_t tsl, bool enable)
{
    tsl->auto_gain = enable;
}

void tsl2561_set_integration_time(TSL2561_handle_t tsl, TSL2561_INTEG_t integ)
{
    bool pwr_off = false;
    if (tsl->config.power == TSL2561_PWR_OFF)
    {
        pwr_off = true;
        tsl2561_enable(tsl);
    }

    tsl->config.integ = integ;
    _tsl2561_write_config(tsl);

    if (pwr_off)
    {
        tsl2561_disable(tsl);
    }
}

void tsl2561_set_gain(TSL2561_handle_t tsl, TSL2561_GAIN_t gain)
{
    bool pwr_off = false;
    if (tsl->config.power == TSL2561_PWR_OFF)
    {
        pwr_off = true;
        tsl2561_enable(tsl);
    }

    tsl->config.gain = gain;
    _tsl2561_write_config(tsl);

    if (pwr_off)
    {
        tsl2561_disable(tsl);
    }
}

void tsl2561_get_lux(TSL2561_handle_t tsl, uint16_t *broadband, uint16_t *ir)
{
    bool valid = false;

    // If Auto gain is disabled get a single reading and continue
    if (!tsl->auto_gain)
    {
        _tsl2561_get_data(tsl, broadband, ir);
        return;
    }

    // read data until we find a valid range
    bool agc_check = false;
    uint16_t _b, _ir;
    uint16_t _hi, _lo;
    do {
        switch (tsl->config.integ)
        {
            case TSL2561_INTEG_13_7ms:
                _hi = TSL2561_AGC_THI_13MS;
                _lo = TSL2561_AGC_TLO_13MS;
                break;
            case TSL2561_INTEG_101ms:
                _hi = TSL2561_AGC_THI_101MS;
                _lo = TSL2561_AGC_TLO_101MS;
                break;
            default:
                _hi = TSL2561_AGC_THI_402MS;
                _lo = TSL2561_AGC_TLO_402MS;
                break;
        }

        _tsl2561_get_data(tsl, &_b, &_ir);

        if (!agc_check)
        {
            if ((_b < _lo) && (tsl->config.gain == TSL2561_GAIN_x1))
            {
                tsl2561_set_gain(tsl, TSL2561_GAIN_x16);
                _tsl2561_get_data(tsl, &_b, &_ir);
                agc_check = true;
            }
            else if ((_b > _hi) && (tsl->config.gain == TSL2561_GAIN_x16))
            {
                tsl2561_set_gain(tsl, TSL2561_GAIN_x1);
                _tsl2561_get_data(tsl, &_b, &_ir);
                agc_check = true;
            }
            else
            {
                *broadband = _b;
                *ir = _ir;
                valid = true;
            }
        }
        else
        {
            *broadband = _b;
            *ir = _ir;
            valid = true;
        }
    }
    while (!valid);
}

void tsl2561_read_calculated_lux(TSL2561_handle_t tsl, uint32_t *lux)
{
    uint16_t b, ir;
    tsl2561_get_lux(tsl, &b, &ir);
    *lux = _tsl2561_calc_lux(tsl->config.gain, tsl->config.integ, b, ir);
}

TSL2561_reg_id_t _tsl2561_read_id(TSL2561_handle_t tsl)
{
    TSL2561_reg_id_t data;
    ESP_ERROR_CHECK(_tsl2561_reg_read8(tsl, TSL2561_REG_ID, (uint8_t*)&data))
    return data;
}

esp_err_t _tsl2561_write_config(TSL2561_handle_t tsl)
{
    TSL2561_reg_control_t reg_control;
    TSL2561_reg_timing_t reg_timing;
    TSL2561_reg_interrupt_t reg_interrupt;

    reg_control.power = tsl->config.power;
    reg_timing.gain = tsl->config.gain;
    reg_timing.integ = tsl->config.integ;
    reg_timing.manual = 0;
    reg_interrupt.intr = tsl->config.intr;
    reg_interrupt.persist = tsl->config.persist;

    _tsl2561_reg_write8(tsl, TSL2561_REG_CONTROL, _tsl2561_reg_control_get(&reg_control));
    _tsl2561_reg_write8(tsl, TSL2561_REG_TIMING, _tsl2561_reg_timing_get(&reg_timing));
    _tsl2561_reg_write8(tsl, TSL2561_REG_INTERRUPT, _tsl2561_reg_interrupt_get(&reg_interrupt));

    return ESP_OK;
}

esp_err_t _tsl2561_get_data(TSL2561_handle_t tsl, uint16_t *broadband, uint16_t *ir)
{
    bool pwr_off = false;
    if (tsl->config.power == TSL2561_PWR_OFF)
    {
        pwr_off = true;
        tsl2561_enable(tsl);
    }

    // wait for ADC to complete
    switch (tsl->config.integ)
    {
        case TSL2561_INTEG_13_7ms:
            vTaskDelay(15 / portTICK_RATE_MS);
            break;
        case TSL2561_INTEG_101ms:
            vTaskDelay(120 / portTICK_RATE_MS);
            break;
        default:
            vTaskDelay(450 / portTICK_RATE_MS);
    }

    _tsl2561_read_ch0(tsl, broadband);
    _tsl2561_read_ch1(tsl, ir);

    if (pwr_off)
    {
        tsl2561_disable(tsl);
    }

    return ESP_OK;
}

esp_err_t _tsl2561_read_ch0(TSL2561_handle_t tsl, uint16_t *data)
{
    uint8_t temp_data[2];

    _tsl2561_reg_read8(tsl, TSL2561_REG_DATA0_L, &temp_data[0]);
    _tsl2561_reg_read8(tsl, TSL2561_REG_DATA0_H, &temp_data[1]);

    *data = (temp_data[1] << 8) | temp_data[0];

    return ESP_OK;
}

esp_err_t _tsl2561_read_ch1(TSL2561_handle_t tsl, uint16_t *data)
{
    uint8_t temp_data[2];

    _tsl2561_reg_read8(tsl, TSL2561_REG_DATA1_L, &temp_data[0]);
    _tsl2561_reg_read8(tsl, TSL2561_REG_DATA1_H, &temp_data[1]);

    *data = (temp_data[1] << 8) | temp_data[0];
    return ESP_OK;
}

TSL2561_reg_command_t _tsl2561_prepare_command(uint8_t reg, bool block)
{
    TSL2561_reg_command_t command;
    command.cmd = 1;
    command.clear = 0;
    command.word = 0;
    command.block = block;
    command.address = reg;
    return command;
}

// region I2C functions

#define ACK_CHECK_EN  0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL       0x0
#define NACK_VAL      0x1
#define LAST_NACK_VAL 0x2

#define TSL_WRITE(addr) ((addr) << 1 | I2C_MASTER_WRITE)
#define TSL_READ(addr) ((addr) << 1 | I2C_MASTER_READ)

esp_err_t _tsl2561_reg_write8(TSL2561_handle_t tsl, uint8_t reg, uint8_t data)
{
    TSL2561_reg_command_t command = _tsl2561_prepare_command(reg, 0);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, TSL_WRITE(tsl->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, _tsl2561_reg_command_get(&command), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(TSL2561_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t _tsl2561_reg_write16(TSL2561_handle_t tsl, uint8_t reg, uint16_t data)
{
    TSL2561_reg_command_t command = _tsl2561_prepare_command(reg, 1);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, TSL_WRITE(tsl->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, _tsl2561_reg_command_get(&command), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write(cmd, (uint8_t*)&data, 2, ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(TSL2561_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t _tsl2561_reg_read8(TSL2561_handle_t tsl, uint8_t reg, uint8_t *data)
{
    TSL2561_reg_command_t command = _tsl2561_prepare_command(reg, 0);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, TSL_WRITE(tsl->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, _tsl2561_reg_command_get(&command), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(TSL2561_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, TSL_READ(tsl->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_read(cmd, data, 1, LAST_NACK_VAL))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(TSL2561_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t _tsl2561_reg_read16(TSL2561_handle_t tsl, uint8_t reg, uint16_t *data)
{
    uint8_t temp_data[2];
    TSL2561_reg_command_t command = _tsl2561_prepare_command(reg, 1);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, TSL_WRITE(tsl->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, _tsl2561_reg_command_get(&command), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(TSL2561_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd))
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, TSL_READ(tsl->addr), ACK_CHECK_EN))
    ESP_ERROR_CHECK(i2c_master_read(cmd, temp_data, 2, LAST_NACK_VAL))
    ESP_ERROR_CHECK(i2c_master_stop(cmd))
    ESP_ERROR_CHECK(i2c_master_cmd_begin(TSL2561_I2C_PORT, cmd, 100 / portTICK_RATE_MS))
    i2c_cmd_link_delete(cmd);

    *data = (temp_data[0] << 8) | temp_data[1];

    return ESP_OK;
}

// endregion
