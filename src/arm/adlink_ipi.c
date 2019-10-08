#include <dirent.h>
#include <mraa/common.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "arm/adlink_ipi.h"
#include "common.h"

#define PLATFORM_NAME_ADLINK_IPI "Adlink IPI - PX30"
#define MRAA_ADLINK_IPI_PINCOUNT 41

static int platform_detected = 0;

static const char* serialdev[] = { "/dev/ttyS0", "/dev/ttyS1" };
static const char* seriallink[] = { "/sys/class/tty/ttyS0", "/sys/class/tty/ttyS1" };

static const char* spilink[] = { "/sys/class/spidev/spidev0.0",
                          "/sys/class/spidev/spidev1.0" };

static const char* i2clink[] = {
    "/sys/class/i2c-dev/i2c-0", "/sys/class/i2c-dev/i2c-1" };

static const char* pwmlink[] = {
    "/sys/class/pwm/pwmchip1", "/sys/class/pwm/pwmchip2" };

mraa_board_t*
mraa_adlink_ipi()
{
    mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof(mraa_board_t));
    if (b == NULL) {
        return NULL;
    }

    platform_detected = 0;
    int i2c0 = -1;
    int i2c1 = -1;
    int spi0 = -1;
    int uart0 = -1;
    int pwm0 = -1;
    int pwm1 = -1;

    //TODO: Handle different Adlink arm variants eg. IMX6, IMX8M
    b->platform_name = PLATFORM_NAME_ADLINK_IPI;
    //platform_detected = PLATFORM_PRO;
    b->phy_pin_count = MRAA_ADLINK_IPI_PINCOUNT;
/*
    if (platform_detected == 0) {
        free(b);
        syslog(LOG_ERR, "mraa: Could not detect platform");
        return NULL;
    }
*/
    int devnum;
    for (devnum = 0; devnum < 2; devnum++) {
        if (mraa_link_targets(seriallink[devnum], "ff030000")) {
            uart0 = devnum;
        }
    }

    for (devnum = 0; devnum < 2; devnum++) {
        if (mraa_link_targets(spilink[devnum], "ff1d0000")) {
            spi0 = devnum;
        }
    }

    for (devnum = 0; devnum < 2; devnum++) {
        if (mraa_link_targets(i2clink[devnum], "ff180000")) {
            i2c0 = devnum;
        }
        if (mraa_link_targets(i2clink[devnum], "ff190000")) {
            i2c1 = devnum;
        }
    }

    for (devnum = 0; devnum < 2; devnum++) {
        if (mraa_link_targets(pwmlink[devnum], "ff208020")) {
            pwm0 = devnum;
        }
        if (mraa_link_targets(pwmlink[devnum], "ff208030")) {
            pwm1 = devnum;
        }
    }

    b->adv_func = (mraa_adv_func_t*) calloc(1, sizeof(mraa_adv_func_t));
    if (b->adv_func == NULL) {
        free(b);
        return NULL;
    }

    b->pins = (mraa_pininfo_t*) calloc(b->phy_pin_count, sizeof(mraa_pininfo_t));
    if (b->pins == NULL) {
        free(b->adv_func);
        free(b);
        return NULL;
    }
/*
    b->adv_func->spi_init_pre = &mraa_adlink_spi_init_pre;
    b->adv_func->i2c_init_pre = &mraa_adlink_i2c_init_pre;
    b->adv_func->gpio_mmap_setup = &mraa_adlink_mmap_setup;
*/
    strncpy(b->pins[0].name, "INVALID", MRAA_PIN_NAME_SIZE);
    b->pins[0].capabilities = (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[1].name, "3V3", MRAA_PIN_NAME_SIZE);
    b->pins[1].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[2].name, "5V", MRAA_PIN_NAME_SIZE);
    b->pins[2].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[3].name, "I2C1_SDA", MRAA_PIN_NAME_SIZE); // GPIO0_C3 
    b->pins[3].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[3].gpio.pinmap = 19;

    strncpy(b->pins[4].name, "5V", MRAA_PIN_NAME_SIZE);
    b->pins[4].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[5].name, "I2C1_SCL", MRAA_PIN_NAME_SIZE); // GPIO0_C2
    b->pins[5].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[5].gpio.pinmap = 18;

    strncpy(b->pins[6].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[6].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    if (pwm0 == 0) {
	    strncpy(b->pins[7].name, "PWM6", MRAA_PIN_NAME_SIZE); // PWM6
	    b->pins[7].capabilities = (mraa_pincapabilities_t){ 1, 0, 1, 0, 0, 0, 0, 0 };
    } else {
	    strncpy(b->pins[7].name, "GPIO3_C4", MRAA_PIN_NAME_SIZE); // GPIO3_C4
	    b->pins[7].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    }
    b->pins[7].gpio.pinmap = 116;
    b->pins[7].pwm.pinmap = 0;
    b->pins[7].pwm.parent_id = 1;
    b->pins[7].pwm.mux_total = 0;

    strncpy(b->pins[8].name, "UART0_TX", MRAA_PIN_NAME_SIZE); // GPIO0_B2
    b->pins[8].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[8].gpio.pinmap = 10;

    strncpy(b->pins[9].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[9].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[10].name, "UART0_RX", MRAA_PIN_NAME_SIZE); // GPIO0_B3
    b->pins[10].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 1 };
    b->pins[10].gpio.pinmap = 11;

    strncpy(b->pins[11].name, "GPIO3_C6", MRAA_PIN_NAME_SIZE); // GPIO3_C6
    b->pins[11].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[11].gpio.pinmap = 118;

    if (pwm1 == 1) {
	    strncpy(b->pins[12].name, "PWM7", MRAA_PIN_NAME_SIZE); // PWM7
	    b->pins[12].capabilities = (mraa_pincapabilities_t){ 1, 0, 1, 0, 0, 0, 0, 0 };
    } else {
	    strncpy(b->pins[12].name, "GPIO3_C5", MRAA_PIN_NAME_SIZE); // GPIO3_C5
	    b->pins[12].capabilities = (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 };
    }
    b->pins[12].gpio.pinmap = 117;
    b->pins[12].pwm.pinmap = 0;
    b->pins[12].pwm.parent_id = 2;
    b->pins[12].pwm.mux_total = 0;

    strncpy(b->pins[13].name, "GPIO3_B3", MRAA_PIN_NAME_SIZE); // GPIO3_B3
    b->pins[13].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[13].gpio.pinmap = 107;

    strncpy(b->pins[14].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[14].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[15].name, "GPIO3_B4", MRAA_PIN_NAME_SIZE); // GPIO3_B4
    b->pins[15].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[15].gpio.pinmap = 108;

    strncpy(b->pins[16].name, "GPIO3_B5", MRAA_PIN_NAME_SIZE); // GPIO3_B5
    b->pins[16].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[16].gpio.pinmap = 109;

    strncpy(b->pins[17].name, "3V3", MRAA_PIN_NAME_SIZE);
    b->pins[17].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[18].name, "GPIO3_D1", MRAA_PIN_NAME_SIZE); // GPIO3_D1
    b->pins[18].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[18].gpio.pinmap = 121;

    strncpy(b->pins[19].name, "SPI0_MOSI", MRAA_PIN_NAME_SIZE); // GPIO1_B4
    b->pins[19].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[19].gpio.pinmap = 44;

    strncpy(b->pins[20].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[20].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[21].name, "SPI0_MISO", MRAA_PIN_NAME_SIZE); // GPIO1_B5
    b->pins[21].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[21].gpio.pinmap = 45;

    strncpy(b->pins[22].name, "GPIO3_D2", MRAA_PIN_NAME_SIZE); // GPIO3_D2
    b->pins[22].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[22].gpio.pinmap = 122;

    strncpy(b->pins[23].name, "SPI0_CLK", MRAA_PIN_NAME_SIZE); // GPIO1_B7
    b->pins[23].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[23].gpio.pinmap = 47;

    strncpy(b->pins[24].name, "SPI0_CSN", MRAA_PIN_NAME_SIZE); // GPIO1_B6
    b->pins[24].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 1, 0, 0, 0 };
    b->pins[24].gpio.pinmap = 46;

    strncpy(b->pins[25].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[25].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[26].name, "SPI0_CS1", MRAA_PIN_NAME_SIZE); // NC 
    b->pins[26].capabilities = (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[27].name, "I2C0_SDA", MRAA_PIN_NAME_SIZE); // GPIO0_B1
    b->pins[27].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[27].gpio.pinmap = 9;

    strncpy(b->pins[28].name, "I2C0_SCL", MRAA_PIN_NAME_SIZE); // GPIO0_B0
    b->pins[28].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 1, 0, 0 };
    b->pins[28].gpio.pinmap = 8;

    strncpy(b->pins[29].name, "EGPIO1_0", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[29].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[29].gpio.pinmap = 496;

    strncpy(b->pins[30].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[30].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[31].name, "EGPIO1_1", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[31].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[31].gpio.pinmap = 497;

    strncpy(b->pins[32].name, "EGPIO1_2", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[32].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[32].gpio.pinmap = 498;

    strncpy(b->pins[33].name, "EGPIO1_3", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[33].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[33].gpio.pinmap = 499;

    strncpy(b->pins[34].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[34].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[35].name, "EGPIO1_4", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[35].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[35].gpio.pinmap = 500;

    strncpy(b->pins[36].name, "EGPIO1_5", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[36].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[36].gpio.pinmap = 501;

    strncpy(b->pins[37].name, "EGPIO1_6", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[37].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[37].gpio.pinmap = 502;

    strncpy(b->pins[38].name, "EGPIO1_7", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[38].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[38].gpio.pinmap = 503;

    strncpy(b->pins[39].name, "GND", MRAA_PIN_NAME_SIZE);
    b->pins[39].capabilities = (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 0 };

    strncpy(b->pins[40].name, "EGPIO2_8", MRAA_PIN_NAME_SIZE); // Expander GPIO
    b->pins[40].capabilities = (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 };
    b->pins[40].gpio.pinmap = 504;

    b->aio_count = 0;
    b->adc_raw = 0;
    b->adc_supported = 0;
    b->pwm_dev_count = 2;
    b->pwm_default_period = 500;
    b->pwm_max_period = 2147483;
    b->pwm_min_period = 1;

    b->gpio_count = 0;
    int i;
    for (i = 0; i < b->phy_pin_count; i++) {
        if (b->pins[i].capabilities.gpio) {
            b->gpio_count++;
        }
    }

    // BUS DEFINITIONS
    b->i2c_bus_count = 0;
    b->def_i2c_bus = 0;
    if (i2c0 >= 0) {
	b->def_i2c_bus = b->i2c_bus_count;
        b->i2c_bus[b->i2c_bus_count].bus_id = i2c0;
        b->i2c_bus[b->i2c_bus_count].sda = 27;
        b->i2c_bus[b->i2c_bus_count].scl = 28;
        b->i2c_bus_count++;
    }

    if (i2c1 >= 0) {
	b->def_i2c_bus = b->i2c_bus_count;
        b->i2c_bus[b->i2c_bus_count].bus_id = i2c1;
        b->i2c_bus[b->i2c_bus_count].sda = 3;
        b->i2c_bus[b->i2c_bus_count].scl = 5;
        b->i2c_bus_count++;
    }

    b->spi_bus_count = 0;
    b->def_spi_bus = 0;
    if (spi0 >= 0) {
        b->spi_bus[b->spi_bus_count].bus_id = spi0;
        b->spi_bus[b->spi_bus_count].slave_s = 0;
        b->spi_bus[b->spi_bus_count].cs = 24;
        b->spi_bus[b->spi_bus_count].mosi = 19;
        b->spi_bus[b->spi_bus_count].miso = 21;
        b->spi_bus[b->spi_bus_count].sclk = 23;
        b->spi_bus_count++;
    }

    b->uart_dev_count = 0;
    b->def_uart_dev = 0;

    if (uart0 >= 0) {
        b->uart_dev[b->uart_dev_count].device_path = (char *)serialdev[uart0];
        b->uart_dev[b->uart_dev_count].rx = 10;
        b->uart_dev[b->uart_dev_count].tx = 8;
        b->uart_dev_count++;
    }

    return b;
}
