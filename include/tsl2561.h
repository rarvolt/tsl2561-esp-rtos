//
// Created by rarvolt on 04.12.2019.
//

#ifndef THUNDER_DETECT_TSL2561_H
#define THUNDER_DETECT_TSL2561_H

#include <stdint.h>
#include <stdbool.h>

// #define TSL2561_PACKAGE_CS


typedef enum
{
    TSL2561_ADDR_SEL_LOW = 0x29,
    TSL2561_ADDR_SEL_FLOAT = 0x39,
    TSL2561_ADDR_SEL_HIGH = 0x49
} TSL2561_ADDR_SEL_t;

typedef enum
{
    TSL2561_PWR_OFF = 0x00,
    TSL2561_PWR_ON = 0x03
} TSL2561_PWR_t;

typedef enum
{
    TSL2561_GAIN_x1 = 0x00,
    TSL2561_GAIN_x16 = 0x01
} TSL2561_GAIN_t;

typedef enum
{
    TSL2561_MANUAL_STOP = 0x00,
    TSL2561_MANUAL_RUN = 0x01
} TSL2561_MANUAL_t;

typedef enum
{
    TSL2561_INTEG_13_7ms = 0x00,
    TSL2561_INTEG_101ms,
    TSL2561_INTEG_402ms,
    TSL2561_INTEG_MAN
} TSL2561_INTEG_t;

typedef enum
{
    TSL2561_INTR_OFF = 0x00,
    TSL2561_INTR_LEVEL,
    TSL2561_INTR_SMBALERT,
    TSL2561_INTR_TEST
} TSL2561_INTR_t;

typedef enum
{
    TSL2561_PERSIST_EVERY_ADC = 0x00,
    TSL2561_PERSIST_THRESH1,
    TSL2561_PERSIST_THRESH2,
    TSL2561_PERSIST_THRESH3,
    TSL2561_PERSIST_THRESH4,
    TSL2561_PERSIST_THRESH5,
    TSL2561_PERSIST_THRESH6,
    TSL2561_PERSIST_THRESH7,
    TSL2561_PERSIST_THRESH8,
    TSL2561_PERSIST_THRESH9,
    TSL2561_PERSIST_THRESH10,
    TSL2561_PERSIST_THRESH11,
    TSL2561_PERSIST_THRESH12,
    TSL2561_PERSIST_THRESH13,
    TSL2561_PERSIST_THRESH14,
    TSL2561_PERSIST_THRESH15,
} TSL2561_PERSIST_t;

typedef struct TSL2561* TSL2561_handle_t;

typedef struct
{
    TSL2561_PWR_t power;
    TSL2561_GAIN_t gain;
    TSL2561_INTEG_t integ;
    TSL2561_INTR_t intr;
    TSL2561_PERSIST_t persist;
} TSL2561_config_t;

TSL2561_handle_t tsl2561_init(TSL2561_ADDR_SEL_t addr, TSL2561_config_t *config);
void tsl2561_set_config(TSL2561_handle_t tsl, TSL2561_config_t *config);
void tsl2561_enable(TSL2561_handle_t tsl);
void tsl2561_disable(TSL2561_handle_t tsl);
void tsl2561_enable_auto_range(TSL2561_handle_t tsl, bool enable);
void tsl2561_set_integration_time(TSL2561_handle_t tsl, TSL2561_INTEG_t integ);
void tsl2561_set_gain(TSL2561_handle_t tsl, TSL2561_GAIN_t gain);
void tsl2561_get_lux(TSL2561_handle_t tsl, uint16_t *broadband, uint16_t *ir);
void tsl2561_read_calculated_lux(TSL2561_handle_t tsl, uint32_t *lux);

#endif //THUNDER_DETECT_TSL2561_H
