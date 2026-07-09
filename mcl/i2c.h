#ifndef I2C_H
#define I2C_H

#include <cstring>

#if defined (PICO)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#elif defined (STM32F4)
#include <mcl/f4/i2c.h>
#elif defined (STM32F767xx)
#include <mcl/f7/i2c.h>
#elif defined (STM32H755xx)
#include <mcl/h7/i2c.h>
#endif

namespace serial {

struct i2c {
    #if defined (STM32)
    I2C_TypeDef *I2Cx;
    GPIO_TypeDef *GPIOx;
    #elif defined (PICO)
    i2c_inst_t *I2Cx;
    #endif
    i2c(int sda, int scl, int freq,
        #if defined (STM32)
        I2C_TypeDef* i2cx, GPIO_TypeDef *gpiox) {
        #elif defined (PICO)
        i2c_inst_t *i2cx) {
        #endif
            I2Cx = i2cx;
            GPIOx = gpiox;
            #if defined (PICO)
            i2c_init(I2Cx, freq);
            gpio_set_function(sda, GPIO_FUNC_I2C);
            gpio_set_function(scl, GPIO_FUNC_I2C);
            gpio_pull_up(sda);
            gpio_pull_up(scl);
            #elif defined (STM32)
            mcl::i2c_init(scl, sda, freq, I2Cx, GPIOx);
            #endif
    }

    #if defined (PICO)
    inline int8_t pico_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
        uint8_t buf[16] = {0};
        buf[0] = reg_addr;
        if (reg_data) {
            memcpy(buf + 1, reg_data, len);
        }
        auto rc = i2c_write_blocking(I2Cx, dev_addr, buf, len + 1, false);
        if (rc == PICO_ERROR_GENERIC) {
            LOG << "pico_i2c_write failed";
            return -1;
        }
        return 0;
    }

    inline int8_t pico_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
        auto rc = i2c_write_blocking(I2Cx, dev_addr, &reg_addr, 1, true);
        if (rc == PICO_ERROR_GENERIC) {
            return 1;
        }
        rc = i2c_read_blocking(I2Cx, dev_addr, reg_data, len, false);
        if (rc == PICO_ERROR_GENERIC) {
            LOG << "pico_i2c_read failed";
            return -1;
        }
        return 0;
    }
    #endif

    #if defined (STM32)
    inline int8_t stm_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, bool sendReg = true) {
        return mcl::i2c_read(dev_addr, reg_addr, reg_data, len, sendReg, I2Cx);
    }

    inline int8_t stm_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, bool sendReg = true) {
        return mcl::i2c_write(dev_addr, reg_addr, reg_data, len, sendReg, I2Cx);
    }
    #endif

    inline int8_t read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, bool sendReg = true) {
        #if defined (PICO)
        return pico_i2c_read(dev_addr, reg_addr, reg_data, len);
        #elif defined (STM32)
        return stm_i2c_read(dev_addr, reg_addr, reg_data, len, sendReg);
        #endif
    }

    inline int8_t write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len, bool sendReg = true) {
        #if defined (PICO)
        return pico_i2c_write(dev_addr, reg_addr, reg_data, len);
        #elif defined (STM32)
        return stm_i2c_write(dev_addr, reg_addr, reg_data, len, sendReg);
        #endif
    }

};

}

#endif