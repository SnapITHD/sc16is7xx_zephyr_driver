/* sc16is7xx.c - SC16IS7XX serial driver */

#define DT_DRV_COMPAT nxp_sc16is7xx

/*
 * Copyright (c) 2022 Opito.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief SC16IS7XX Serial Driver - RS-485 Configuration
 *
 * This is the driver for the NXP SC16IS7XX I2C UART Chip
 * Configured for RS-485 operation with GPIO-controlled transceivers
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/types.h>

#include <zephyr/init.h>
#include <zephyr/toolchain.h>
#include <zephyr/linker/sections.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/pm/policy.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include "sc16is7xx.h"
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(sc16is7xx, LOG_LEVEL_INF);

/* register definitions */

#define REG_THR 0x00  /* Transmitter holding reg.       */
#define REG_RDR 0x00  /* Receiver data reg.             */
#define REG_BRDL 0x00 /* Baud rate divisor (LSB)        */
#define REG_BRDH 0x01 /* Baud rate divisor (MSB)        */
#define REG_IER 0x01  /* Interrupt enable reg.          */
#define REG_IIR 0x02  /* Interrupt ID reg.              */
#define REG_FCR 0x02  /* FIFO control reg.              */
#define REG_LCR 0x03  /* Line control reg.              */
#define REG_MCR 0x04  /* Modem control reg.             */
#define REG_LSR 0x05  /* Line status reg.               */
#define REG_MSR 0x06  /* Modem status reg.              */
#define REG_SPR 0x07  /* Scratch Pad Register			*/
#define REG_EFR 0x02  /* Enhanced Features Register     */
#define REG_TXLVL 0x08
#define REG_RXLVL 0x09
#define REG_IODIR 0x0a
#define REG_IOSTATE 0x0b
#define REG_IOINTENA 0x0c
#define REG_IOCONTROL 0x0e
#define REG_EFCR 0x0f

/* equates for interrupt enable register */

#define IER_RXRDY 0x01 /* receiver data ready */
#define IER_TBE 0x02   /* transmit bit enable */
#define IER_LSR 0x04   /* line status interrupts */
#define IER_MSI 0x08   /* modem status interrupts */

/* equates for interrupt identification register */

#define IIR_MSTAT 0x00 /* modem status interrupt  */
#define IIR_NIP 0x01   /* no interrupt pending    */
#define IIR_THRE 0x02  /* transmit holding register empty interrupt */
#define IIR_RBRF 0x04  /* receiver buffer register full interrupt */
#define IIR_LS 0x06    /* receiver line status interrupt */
#define IIR_MASK 0x07  /* interrupt id bits mask  */
#define IIR_ID 0x06    /* interrupt ID mask without NIP */
#define IIR_FE 0xC0    /* FIFO mode enabled */
#define IIR_CH 0x0C    /* Character timeout*/

/* equates for FIFO control register */

#define FCR_FIFO 0x01    /* enable XMIT and RCVR FIFO */
#define FCR_RCVRCLR 0x02 /* clear RCVR FIFO */
#define FCR_XMITCLR 0x04 /* clear XMIT FIFO */

/* RCVR FIFO interrupt levels: trigger interrupt with this bytes in FIFO */
#define FCR_FIFO_8 0x00  /* 8 byte in RCVR FIFO */
#define FCR_FIFO_16 0x40 /* 16 bytes in RCVR FIFO */
#define FCR_FIFO_56 0x80 /* 56 bytes in RCVR FIFO */
#define FCR_FIFO_60 0xC0 /*  60 bytes in RCVR FIFO */
#define FCR_FIFO_64 0xC0 /* Enable 64 bytes FIFO */

/* FCR register bits */
#define SC16IS7XX_FCR_FIFO_BIT (1 << 0)    /* Enable FIFO */
#define SC16IS7XX_FCR_RXRESET_BIT (1 << 1) /* Reset RX FIFO */
#define SC16IS7XX_FCR_TXRESET_BIT (1 << 2) /* Reset TX FIFO */
#define SC16IS7XX_FCR_RXLVLL_BIT (1 << 6)  /* RX Trigger level LSB */
#define SC16IS7XX_FCR_RXLVLH_BIT (1 << 7)  /* RX Trigger level MSB */

/* FCR register bits - write only if (EFR[4] == 1) */
#define SC16IS7XX_FCR_TXLVLL_BIT (1 << 4) /* TX Trigger level LSB */
#define SC16IS7XX_FCR_TXLVLH_BIT (1 << 5) /* TX Trigger level MSB */

/* GPIO RS-485 control bits - Based on actual hardware schematic */

/* Channel A (MODBUS Port 1) - Transceiver control */
#define CHANNEL_A_DE (1 << 0) // GPIO0 - Driver Enable (active HIGH to transmit)
#define CHANNEL_A_RE (1 << 1) // GPIO1 - Receiver Enable (active LOW to receive)

/* Channel B (MODBUS Port 2) - Transceiver control */
#define CHANNEL_B_DE (1 << 2) // GPIO2 - Driver Enable (active HIGH to transmit)
#define CHANNEL_B_RE (1 << 3) // GPIO3 - Receiver Enable (active LOW to receive)

/* Power control pins */
#define PORT_A_POWER_EN (1 << 4) // GPIO4 - Enable ground for Port A (active HIGH)
#define PORT_B_POWER_EN (1 << 5) // GPIO5 - Enable ground for Port B (active HIGH)
#define POWER_5V_EN (1 << 6)     // GPIO6 - Enable 5V rail (active HIGH)
#define POWER_12V_EN (1 << 7)    // GPIO7 - Enable 12V rail (active HIGH)

/* Masks for transceiver control per channel */
#define CHANNEL_A_TRANSCEIVER_MASK (CHANNEL_A_DE | CHANNEL_A_RE)
#define CHANNEL_B_TRANSCEIVER_MASK (CHANNEL_B_DE | CHANNEL_B_RE)

/* All power control bits */
#define POWER_CONTROL_MASK (PORT_A_POWER_EN | PORT_B_POWER_EN | POWER_5V_EN | POWER_12V_EN)

/* EFCR register bit definitions for RS-485 auto direction control */
#define EFCR_AUTO_RS485_DIR (1 << 4) // bit 4: Enable automatic RTS direction control
#define EFCR_RTS_INVERT (1 << 5)     // bit 5: RTS=1 during TX (for DE pin control)

/* constants for line control register */

#define LCR_CS5 0x00        /* 5 bits data size */
#define LCR_CS6 0x01        /* 6 bits data size */
#define LCR_CS7 0x02        /* 7 bits data size */
#define LCR_CS8 0x03        /* 8 bits data size */
#define LCR_2_STB 0x04      /* 2 stop bits */
#define LCR_1_STB 0x00      /* 1 stop bit */
#define LCR_PEN 0x08        /* parity enable */
#define LCR_PDIS 0x00       /* parity disable */
#define LCR_EPS 0x10        /* even parity select */
#define LCR_SP 0x20         /* stick parity select */
#define LCR_SBRK 0x40       /* break control bit */
#define LCR_DLAB 0x80       /* divisor latch access enable */
#define LCR_EFR_ACCESS 0xBF /* Enhanced features access enable */

/* constants for the modem control register */
#define MCR_DTR_BIT (1 << 0)    /* DTR complement */
#define MCR_RTS_BIT (1 << 1)    /* RTS complement */
#define MCR_TCRTLR_BIT (1 << 2) /* TCR/TLR register enable */
#define MCR_LOOP_BIT (1 << 4)   /* Enable loopback test mode */
#define MCR_XONANY_BIT (1 << 5) /* Enable Xon Any */
#define MCR_IRDA_BIT (1 << 6)   /* Enable IrDA mode */
#define MCR_CLKSEL_BIT (1 << 7) /* Divide clock by 4 */

/* Missing definitions for line control */
#define MCR_DTR 0x01
#define MCR_RTS 0x02

/* constants for line status register */

#define LSR_RXRDY 0x01    /* receiver data available */
#define LSR_OE 0x02       /* overrun error */
#define LSR_PE 0x04       /* parity error */
#define LSR_FE 0x08       /* framing error */
#define LSR_BI 0x10       /* break interrupt */
#define LSR_EOB_MASK 0x1E /* Error or Break mask */
#define LSR_THRE 0x20     /* transmit holding register empty */
#define LSR_TEMT 0x40     /* transmitter empty */

/* constants for modem status register */

#define MSR_DCTS 0x01 /* cts change */
#define MSR_DDSR 0x02 /* dsr change */
#define MSR_DRI 0x04  /* ring change */
#define MSR_DDCD 0x08 /* data carrier change */
#define MSR_CTS 0x10  /* complement of cts */
#define MSR_DSR 0x20  /* complement of dsr */
#define MSR_RI 0x40   /* complement of ring signal */
#define MSR_DCD 0x80  /* complement of dcd */

/* EFCR register bits */
#define EFCR_9BIT_MODE_BIT (1 << 0)  /* Enable 9-bit or Multidrop mode (RS485) */
#define EFCR_RXDISABLE_BIT (1 << 1)  /* Disable receiver */
#define EFCR_TXDISABLE_BIT (1 << 2)  /* Disable transmitter */
#define EFCR_AUTO_RS485_BIT (1 << 4) /* Auto RS485 RTS direction */
#define EFCR_RTS_INVERT_BIT (1 << 5) /* RTS output inversion */
#define EFCR_IRDA_MODE_BIT (1 << 7)  /* IrDA mode */

/* EFR register bits */
#define EFR_AUTORTS_BIT (1 << 6)      /* Auto RTS flow ctrl enable */
#define EFR_AUTOCTS_BIT (1 << 7)      /* Auto CTS flow ctrl enable */
#define EFR_XOFF2_DETECT_BIT (1 << 5) /* Enable Xoff2 detection */
#define EFR_ENABLE_BIT (1 << 4)       /* Enable enhanced functions */
#define EFR_SWFLOW3_BIT (1 << 3)      /* SWFLOW bit 3 */
#define EFR_SWFLOW2_BIT (1 << 2)      /* SWFLOW bit 2 */
#define EFR_SWFLOW1_BIT (1 << 1)      /* SWFLOW bit 1 */
#define EFR_SWFLOW0_BIT (1 << 0)      /* SWFLOW bit 0 */

#define reg_interval(x) 3

#define THR(dev) (REG_THR << reg_interval(dev) | get_port(dev))
#define RDR(dev) (REG_RDR << reg_interval(dev) | get_port(dev))
#define BRDL(dev) (REG_BRDL << reg_interval(dev) | get_port(dev))
#define BRDH(dev) (REG_BRDH << reg_interval(dev) | get_port(dev))
#define IER(dev) (REG_IER << reg_interval(dev) | get_port(dev))
#define IIR(dev) (REG_IIR << reg_interval(dev) | get_port(dev))
#define FCR(dev) (REG_FCR << reg_interval(dev) | get_port(dev))
#define LCR(dev) (REG_LCR << reg_interval(dev) | get_port(dev))
#define MCR(dev) (REG_MCR << reg_interval(dev) | get_port(dev))
#define LSR(dev) (REG_LSR << reg_interval(dev) | get_port(dev))
#define MSR(dev) (REG_MSR << reg_interval(dev) | get_port(dev))
#define SPR(dev) (REG_SPR << reg_interval(dev) | get_port(dev))
#define EFR(dev) (REG_EFR << reg_interval(dev) | get_port(dev))
#define TXLVL(dev) (REG_TXLVL << reg_interval(dev) | get_port(dev))
#define RXLVL(dev) (REG_RXLVL << reg_interval(dev) | get_port(dev))
#define IODIR(dev) (REG_IODIR << reg_interval(dev))
#define IOSTATE(dev) (REG_IOSTATE << reg_interval(dev))
#define IOINTENA(dev) (REG_IOINTENA << reg_interval(dev))
#define IOCONTROL(dev) (REG_IOCONTROL << reg_interval(dev))
#define EFCR(dev) (REG_EFCR << reg_interval(dev) | get_port(dev))

#define IIRC(dev) (((struct sc16is7xx_dev_data *)(dev)->data)->iir_cache)

int sc16is17xx_reg_read(const struct device *dev, uint8_t reg);
int sc16is17xx_reg_write(const struct device *dev, uint8_t reg, uint8_t val);

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
static void sc16is7xx_isr(const struct device *dev);
static int sc16is7xx_irq_update(const struct device *dev);
#endif

#define INBYTE(dev, x) sc16is17xx_reg_read(dev, x)
#define OUTBYTE(dev, x, d) sc16is17xx_reg_write(dev, x, d)

/* Serial interface modes */
enum sc16is7xx_serial_mode
{
    SC16IS7XX_MODE_RS232 = 0, /* Standard RS-232 */
    SC16IS7XX_MODE_RS485 = 1, /* RS-485 with GPIO direction control */
    SC16IS7XX_MODE_RS422 = 2, /* RS-422 full-duplex */
};

/* device config */
struct sc16is7xx_device_config
{
    uint32_t port;
    uint8_t serport_num;
    struct i2c_dt_spec bus;
    uint32_t sys_clk_freq;
    enum sc16is7xx_serial_mode mode;
#if defined(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN)
    struct gpio_dt_spec int_gpio;
#endif
};

/** Device data structure */
struct sc16is7xx_dev_data
{
    struct uart_config uart_config;
    struct k_spinlock lock;
    uint8_t fifo_size;

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
    const struct device *instance;
    struct gpio_callback gpio_callback;
    struct k_work interrupt_worker;
    bool interrupt_active;
    uint8_t iir_cache;
    uart_irq_callback_user_data_t cb;
    void *cb_data;
    bool tx_stream_on;
#endif
};

/* ============================================================================
 * CHIP-LEVEL STATE MANAGEMENT (Per-Chip, Independent of UARTs)
 * ============================================================================ */

#define MAX_SC16IS7XX_CHIPS 4 /* Support up to 4 chips (addresses 0x48-0x4B) */

/**
 * @brief Per-chip state structure
 *
 * Each physical SC16IS7XX chip maintains its own independent state,
 * regardless of how many UART channels it has.
 */
struct sc16is7xx_chip_state
{
    bool initialized;       /* Has this chip been initialized? */
    uint8_t gpio_state;     /* Current GPIO state for this chip */
    uint16_t i2c_addr;      /* 7-bit I2C address (0x48, 0x49, etc.) */
    struct i2c_dt_spec bus; /* I2C bus spec for register access */
};

/* Global chip state array - protected by mutex */
static struct k_mutex sc16is7xx_chip_mutex;
static bool sc16is7xx_chip_mutex_initialized = false;
static struct sc16is7xx_chip_state sc16is7xx_chips[MAX_SC16IS7XX_CHIPS] = {0};

/**
 * @brief Get chip index from I2C address
 *
 * @param addr 7-bit I2C address
 * @return Chip index (0-3) or -1 if not found
 */
static int sc16is7xx_get_chip_index(uint16_t addr)
{
    /* First, check if this address is already tracked */
    for (int i = 0; i < MAX_SC16IS7XX_CHIPS; i++)
    {
        if (sc16is7xx_chips[i].i2c_addr == addr)
        {
            return i;
        }
    }

    /* Not found, allocate a new slot */
    for (int i = 0; i < MAX_SC16IS7XX_CHIPS; i++)
    {
        if (sc16is7xx_chips[i].i2c_addr == 0)
        {
            sc16is7xx_chips[i].i2c_addr = addr;
            return i;
        }
    }

    /* No free slots */
    return -1;
}

/**
 * @brief Get chip state structure
 *
 * @param dev Device structure (from any UART channel on the chip)
 * @return Pointer to chip state or NULL on error
 */
static struct sc16is7xx_chip_state *sc16is7xx_get_chip_state(const struct device *dev)
{
    const struct sc16is7xx_device_config *const config = dev->config;
    int chip_idx = sc16is7xx_get_chip_index(config->bus.addr);

    if (chip_idx < 0)
    {
        LOG_ERR("Too many SC16IS7XX chips! Max supported: %d", MAX_SC16IS7XX_CHIPS);
        return NULL;
    }

    /* Store bus spec for later use if not already set */
    if (sc16is7xx_chips[chip_idx].bus.bus == NULL)
    {
        sc16is7xx_chips[chip_idx].bus = config->bus;
    }

    return &sc16is7xx_chips[chip_idx];
}

/**
 * @brief Chip-level register read (uses chip state, not UART device)
 */
static uint8_t sc16is7xx_chip_reg_read(struct sc16is7xx_chip_state *chip, uint8_t reg)
{
    uint8_t data;
    i2c_burst_read_dt(&chip->bus, reg, &data, 1);
    return data;
}

/**
 * @brief Chip-level register write (uses chip state, not UART device)
 */
static int sc16is7xx_chip_reg_write(struct sc16is7xx_chip_state *chip, uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2];
    tx_buf[0] = reg;
    tx_buf[1] = val;
    return i2c_write_dt(&chip->bus, tx_buf, sizeof(tx_buf));
}

/**
 * @brief Initialize chip-level hardware (GPIO, power rails, reset)
 */
static int sc16is7xx_chip_hardware_init(const struct device *dev)
{
    const struct sc16is7xx_device_config *const config = dev->config;
    int ret = 0;

    /* Initialize mutex on first call (global, one-time) */
    if (!sc16is7xx_chip_mutex_initialized)
    {
        k_mutex_init(&sc16is7xx_chip_mutex);
        sc16is7xx_chip_mutex_initialized = true;
        LOG_DBG("Global chip mutex initialized");
    }

    k_mutex_lock(&sc16is7xx_chip_mutex, K_FOREVER);

    /* Get this chip's state */
    struct sc16is7xx_chip_state *chip = sc16is7xx_get_chip_state(dev);
    if (!chip)
    {
        k_mutex_unlock(&sc16is7xx_chip_mutex);
        return -ENOMEM;
    }

    if (!chip->initialized)
    {
        LOG_INF("=== SC16IS7XX Chip @ 0x%02X Hardware Initialization ===",
                config->bus.addr);

        /* Software reset - affects BOTH channels on THIS chip */
        LOG_INF("Chip @ 0x%02X: Performing hardware reset...", config->bus.addr);

        /* GPIO registers don't need channel offset - they're shared */
        sc16is7xx_chip_reg_write(chip, REG_IOCONTROL << 3, 0x08);
        k_sleep(K_MSEC(10));

        /* Configure all GPIO pins as outputs */
        sc16is7xx_chip_reg_write(chip, REG_IODIR << 3, 0xFF);

        /* Initialize this chip's GPIO state:
         * - Power rails OFF
         * - Port power OFF
         * - Transceivers in receive mode (DE=LOW, RE=LOW)
         */
        chip->gpio_state = 0x00;
        sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);

        LOG_INF("Chip @ 0x%02X: GPIO initialized to 0x%02X (all OFF, RX mode)",
                config->bus.addr, chip->gpio_state);

        /* === POWER SEQUENCING TEST === */
        LOG_INF("=== Power Sequencing Test: Chip @ 0x%02X ===", config->bus.addr);

        /* Step 1: Enable 12V rail */
        LOG_INF("Chip @ 0x%02X: Step 1 - Enabling 12V rail...", config->bus.addr);
        chip->gpio_state |= POWER_12V_EN;
        sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);
        LOG_INF("  12V ON - GPIO state = 0x%02X", chip->gpio_state);
        k_sleep(K_MSEC(500));

        /* Step 2: Enable 5V rail */
        LOG_INF("Chip @ 0x%02X: Step 2 - Enabling 5V rail...", config->bus.addr);
        chip->gpio_state |= POWER_5V_EN;
        sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);
        LOG_INF("  5V ON - GPIO state = 0x%02X", chip->gpio_state);
        k_sleep(K_MSEC(500));

        /* Step 3: Enable Port A power */
        LOG_INF("Chip @ 0x%02X: Step 3 - Enabling Port A power...", config->bus.addr);
        chip->gpio_state |= PORT_A_POWER_EN;
        sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);
        LOG_INF("  Port A ON - GPIO state = 0x%02X", chip->gpio_state);
        k_sleep(K_MSEC(500));

        /* Step 4: Enable Port B power */
        LOG_INF("Chip @ 0x%02X: Step 4 - Enabling Port B power...", config->bus.addr);
        chip->gpio_state |= PORT_B_POWER_EN;
        sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);
        LOG_INF("  Port B ON - GPIO state = 0x%02X", chip->gpio_state);
        k_sleep(K_MSEC(500));

        LOG_INF("=== Power Sequencing Complete: Chip @ 0x%02X ===", config->bus.addr);
        LOG_INF("All power rails enabled for chip @ 0x%02X", config->bus.addr);
        LOG_INF("Final GPIO state = 0x%02X", chip->gpio_state);

        /* Verify by reading back */
        uint8_t gpio_readback = sc16is7xx_chip_reg_read(chip, REG_IOSTATE << 3);
        LOG_INF("GPIO readback = 0x%02X", gpio_readback);

        if (gpio_readback != chip->gpio_state)
        {
            LOG_WRN("Chip @ 0x%02X: GPIO readback mismatch! Expected 0x%02X, got 0x%02X",
                    config->bus.addr, chip->gpio_state, gpio_readback);
        }

        chip->initialized = true;
        LOG_INF("=== Chip @ 0x%02X Hardware Initialization Complete ===",
                config->bus.addr);
    }
    else
    {
        LOG_DBG("Chip @ 0x%02X: Already initialized, skipping hardware init",
                config->bus.addr);
    }

    k_mutex_unlock(&sc16is7xx_chip_mutex);

    return ret;
}

/**
 * @brief Control GPIO state for a specific chip
 */
int sc16is7xx_chip_gpio_control(uint16_t chip_addr, uint8_t gpio_mask, uint8_t gpio_value)
{
    k_mutex_lock(&sc16is7xx_chip_mutex, K_FOREVER);

    /* Find the chip */
    int chip_idx = sc16is7xx_get_chip_index(chip_addr);
    if (chip_idx < 0)
    {
        k_mutex_unlock(&sc16is7xx_chip_mutex);
        return -ENODEV;
    }

    struct sc16is7xx_chip_state *chip = &sc16is7xx_chips[chip_idx];

    if (!chip->initialized)
    {
        k_mutex_unlock(&sc16is7xx_chip_mutex);
        return -EINVAL;
    }

    /* Modify only the specified bits */
    chip->gpio_state = (chip->gpio_state & ~gpio_mask) | (gpio_value & gpio_mask);
    sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);

    LOG_DBG("Chip @ 0x%02X: GPIO updated to 0x%02X (mask=0x%02X, value=0x%02X)",
            chip_addr, chip->gpio_state, gpio_mask, gpio_value);

    k_mutex_unlock(&sc16is7xx_chip_mutex);

    return 0;
}

/**
 * @brief Configure transceiver direction control for a specific UART channel
 */
static void sc16is7xx_set_transceiver_direction(const struct device *dev, bool transmit)
{
    const struct sc16is7xx_device_config *const dev_cfg = dev->config;

    k_mutex_lock(&sc16is7xx_chip_mutex, K_FOREVER);

    struct sc16is7xx_chip_state *chip = sc16is7xx_get_chip_state(dev);
    if (!chip)
    {
        k_mutex_unlock(&sc16is7xx_chip_mutex);
        return;
    }

    if (dev_cfg->serport_num == 0)
    {
        /* Channel A */
        if (transmit)
        {
            chip->gpio_state |= CHANNEL_A_DE;
            chip->gpio_state |= CHANNEL_A_RE;
        }
        else
        {
            chip->gpio_state &= ~CHANNEL_A_DE;
            chip->gpio_state &= ~CHANNEL_A_RE;
        }
    }
    else
    {
        /* Channel B */
        if (transmit)
        {
            chip->gpio_state |= CHANNEL_B_DE;
            chip->gpio_state |= CHANNEL_B_RE;
        }
        else
        {
            chip->gpio_state &= ~CHANNEL_B_DE;
            chip->gpio_state &= ~CHANNEL_B_RE;
        }
    }

    sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);

    k_mutex_unlock(&sc16is7xx_chip_mutex);
}

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN

static void sc16is7xx_interrupt_worker(struct k_work *work)
{
    struct sc16is7xx_dev_data *const drv_data = CONTAINER_OF(
        work, struct sc16is7xx_dev_data, interrupt_worker);

    sc16is7xx_isr(drv_data->instance);
}

static void sc16is7xx_interrupt_callback(const struct device *dev,
                                         struct gpio_callback *cb,
                                         gpio_port_pins_t pins)
{
    struct sc16is7xx_dev_data *const drv_data =
        CONTAINER_OF(cb, struct sc16is7xx_dev_data, gpio_callback);

    ARG_UNUSED(pins);

    k_work_submit(&drv_data->interrupt_worker);
}

#endif

int sc16is17xx_reg_read(const struct device *dev, uint8_t reg)
{
    const struct sc16is7xx_device_config *const config = dev->config;
    uint8_t data;
    i2c_burst_read_dt(&config->bus, reg, &data, 1);
    return data;
}

int sc16is17xx_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct sc16is7xx_device_config *const config = dev->config;

    uint8_t tx_buf[2];
    tx_buf[0] = reg;
    tx_buf[1] = val;

    return i2c_write_dt(&config->bus, tx_buf, sizeof(tx_buf));
}

static const struct uart_driver_api sc16is7xx_driver_api;

static inline uintptr_t get_port(const struct device *dev)
{
    const struct sc16is7xx_device_config *config = dev->config;

    return config->serport_num << 1;
}

static void set_baud_rate(const struct device *dev, uint32_t baud_rate, uint32_t pclk)
{
    struct sc16is7xx_dev_data *const dev_data = dev->data;
    uint32_t divisor;
    uint8_t lcr_cache;

    if ((baud_rate != 0U) && (pclk != 0U))
    {
        divisor = pclk / (baud_rate * 16);

        lcr_cache = INBYTE(dev, LCR(dev));

        OUTBYTE(dev, LCR(dev), LCR_DLAB);

        OUTBYTE(dev, BRDL(dev), (unsigned char)(divisor & 0xff));
        OUTBYTE(dev, BRDH(dev), (unsigned char)((divisor >> 8) & 0xff));

        OUTBYTE(dev, LCR(dev), lcr_cache);

        OUTBYTE(dev, LCR(dev), LCR_EFR_ACCESS);
        OUTBYTE(dev, EFR(dev), 0x00);

        OUTBYTE(dev, LCR(dev), lcr_cache);

        dev_data->uart_config.baudrate = baud_rate;
    }
}

/**
 * @brief Configure UART channel mode (RS-232, RS-485, RS-422)
 */
static void configure_uart_mode(const struct device *dev)
{
    const struct sc16is7xx_device_config *const dev_cfg = dev->config;
    uint8_t efcr = 0;

    efcr = INBYTE(dev, EFCR(dev));

    switch (dev_cfg->mode)
    {
    case SC16IS7XX_MODE_RS485:
        LOG_INF("Chip @ 0x%02X, Channel %d: Configuring for RS-485 mode",
                dev_cfg->bus.addr, dev_cfg->serport_num);

        efcr &= ~EFCR_AUTO_RS485_DIR;
        efcr &= ~EFCR_RTS_INVERT;
        efcr &= ~EFCR_9BIT_MODE_BIT;
        OUTBYTE(dev, EFCR(dev), efcr);

        sc16is7xx_set_transceiver_direction(dev, false);

        LOG_INF("Channel %d RS-485 configured: EFCR=0x%02X", dev_cfg->serport_num, efcr);
        break;

    case SC16IS7XX_MODE_RS422:
        LOG_INF("Chip @ 0x%02X, Channel %d: Configuring for RS-422 mode",
                dev_cfg->bus.addr, dev_cfg->serport_num);

        efcr &= ~EFCR_AUTO_RS485_DIR;
        efcr &= ~EFCR_RTS_INVERT;
        efcr &= ~EFCR_9BIT_MODE_BIT;
        OUTBYTE(dev, EFCR(dev), efcr);

        k_mutex_lock(&sc16is7xx_chip_mutex, K_FOREVER);
        struct sc16is7xx_chip_state *chip = sc16is7xx_get_chip_state(dev);
        if (chip)
        {
            if (dev_cfg->serport_num == 0)
            {
                chip->gpio_state |= CHANNEL_A_DE;
                chip->gpio_state &= ~CHANNEL_A_RE;
            }
            else
            {
                chip->gpio_state |= CHANNEL_B_DE;
                chip->gpio_state &= ~CHANNEL_B_RE;
            }
            sc16is7xx_chip_reg_write(chip, REG_IOSTATE << 3, chip->gpio_state);
        }
        k_mutex_unlock(&sc16is7xx_chip_mutex);

        LOG_INF("Channel %d RS-422 configured: EFCR=0x%02X", dev_cfg->serport_num, efcr);
        break;

    case SC16IS7XX_MODE_RS232:
    default:
        LOG_INF("Chip @ 0x%02X, Channel %d: Configuring for RS-232 mode",
                dev_cfg->bus.addr, dev_cfg->serport_num);

        efcr &= ~EFCR_AUTO_RS485_DIR;
        efcr &= ~EFCR_RTS_INVERT;
        efcr &= ~EFCR_9BIT_MODE_BIT;
        OUTBYTE(dev, EFCR(dev), efcr);

        LOG_INF("Channel %d RS-232 configured: EFCR=0x%02X", dev_cfg->serport_num, efcr);
        break;
    }
}

static int sc16is7xx_configure(const struct device *dev,
                               const struct uart_config *cfg)
{
    struct sc16is7xx_dev_data *const dev_data = dev->data;
    const struct sc16is7xx_device_config *const dev_cfg = dev->config;

    int ret = 0;
    uint32_t pclk = 0U;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);

    configure_uart_mode(dev);

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
    dev_data->iir_cache = 0U;
#endif

    struct uart_config uart_cfg;

    switch (cfg->data_bits)
    {
    case UART_CFG_DATA_BITS_5:
        uart_cfg.data_bits = LCR_CS5;
        break;
    case UART_CFG_DATA_BITS_6:
        uart_cfg.data_bits = LCR_CS6;
        break;
    case UART_CFG_DATA_BITS_7:
        uart_cfg.data_bits = LCR_CS7;
        break;
    case UART_CFG_DATA_BITS_8:
        uart_cfg.data_bits = LCR_CS8;
        break;
    default:
        ret = -ENOTSUP;
        goto out;
    }

    switch (cfg->stop_bits)
    {
    case UART_CFG_STOP_BITS_1:
        uart_cfg.stop_bits = LCR_1_STB;
        break;
    case UART_CFG_STOP_BITS_2:
        uart_cfg.stop_bits = LCR_2_STB;
        break;
    default:
        ret = -ENOTSUP;
        goto out;
    }

    switch (cfg->parity)
    {
    case UART_CFG_PARITY_NONE:
        uart_cfg.parity = LCR_PDIS;
        break;
    case UART_CFG_PARITY_EVEN:
        uart_cfg.parity = LCR_EPS | LCR_PEN;
        break;
    case UART_CFG_PARITY_ODD:
        uart_cfg.parity = LCR_PEN;
        break;
    default:
        ret = -ENOTSUP;
        goto out;
    }

    dev_data->uart_config = *cfg;

    uint8_t lcr_value = uart_cfg.data_bits | uart_cfg.stop_bits | uart_cfg.parity;
    OUTBYTE(dev, LCR(dev), lcr_value);

    pclk = dev_cfg->sys_clk_freq;
    set_baud_rate(dev, cfg->baudrate, pclk);

    OUTBYTE(dev, FCR(dev),
            FCR_FIFO | FCR_FIFO_8 | FCR_RCVRCLR | FCR_XMITCLR | FCR_FIFO_64);

    if ((INBYTE(dev, IIR(dev)) & IIR_FE) == IIR_FE)
    {
        dev_data->fifo_size = 64;
    }
    else
    {
        dev_data->fifo_size = 1;
    }

    INBYTE(dev, RDR(dev));

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
    OUTBYTE(dev, IER(dev), 0x00);
#endif

out:
    k_spin_unlock(&dev_data->lock, key);
    return ret;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int sc16is7xx_config_get(const struct device *dev,
                                struct uart_config *cfg)
{
    struct sc16is7xx_dev_data *data = dev->data;

    cfg->baudrate = data->uart_config.baudrate;
    cfg->parity = data->uart_config.parity;
    cfg->stop_bits = data->uart_config.stop_bits;
    cfg->data_bits = data->uart_config.data_bits;
    cfg->flow_ctrl = data->uart_config.flow_ctrl;

    return 0;
}
#endif

static int sc16is7xx_init(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    const struct sc16is7xx_device_config *const config = dev->config;
    int ret;

    if (!device_is_ready(config->bus.bus))
    {
        LOG_ERR("I2C bus %s is not ready", config->bus.bus->name);
        return -ENODEV;
    }

    /* === STEP 1: Initialize chip hardware (once per chip) === */
    ret = sc16is7xx_chip_hardware_init(dev);
    if (ret != 0)
    {
        LOG_ERR("Chip hardware initialization failed: ret=%d", ret);
        return ret;
    }

    /* === STEP 2: Test UART channel communication === */
    ret = sc16is17xx_reg_write(dev, SPR(dev), 0xAB);
    if (ret < 0)
    {
        LOG_ERR("Failed to write SPR register!");
        return ret;
    }

    uint8_t ping_req = sc16is17xx_reg_read(dev, SPR(dev));
    if (ping_req == 0xAB)
    {
        LOG_INF("Chip @ 0x%02X, Channel %d: Communication OK",
                config->bus.addr, config->serport_num);
    }
    else
    {
        LOG_ERR("Chip @ 0x%02X, Channel %d: Communication failed (expected 0xAB, got 0x%02X)",
                config->bus.addr, config->serport_num, ping_req);
        return -EIO;
    }

    /* === STEP 3: Configure UART channel parameters === */
    ret = sc16is7xx_configure(dev, &data->uart_config);
    if (ret != 0)
    {
        LOG_ERR("sc16is7xx_configure failed: ret=%d", ret);
        return ret;
    }

#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
    data->instance = dev;

    k_work_init(&data->interrupt_worker, sc16is7xx_interrupt_worker);

    if (!device_is_ready(config->int_gpio.port))
    {
        LOG_ERR("sc16is7xx[0x%X]: interrupt GPIO not ready",
                config->bus.addr);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
    if (ret != 0)
    {
        LOG_ERR("sc16is7xx[0x%02X]: failed to configure interrupt pin %d (%d)",
                config->bus.addr, config->int_gpio.pin, ret);
        return ret;
    }

    gpio_init_callback(&data->gpio_callback,
                       sc16is7xx_interrupt_callback,
                       BIT(config->int_gpio.pin));
    gpio_add_callback(config->int_gpio.port, &data->gpio_callback);

    ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_FALLING);
    if (ret != 0)
    {
        LOG_ERR("Failed to configure interrupt (%d)", ret);
        return ret;
    }
#endif

    LOG_INF("Chip @ 0x%02X, Channel %d: Initialization complete",
            config->bus.addr, config->serport_num);
    return 0;
}

static int sc16is7xx_poll_in(const struct device *dev, unsigned char *c)
{
    struct sc16is7xx_dev_data *data = dev->data;
    int ret = -1;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    if ((INBYTE(dev, LSR(dev)) & LSR_RXRDY) != 0)
    {
        *c = INBYTE(dev, RDR(dev));
        ret = 0;
    }

    k_spin_unlock(&data->lock, key);

    return ret;
}

static void sc16is7xx_poll_out(const struct device *dev, unsigned char c)
{
    struct sc16is7xx_dev_data *data = dev->data;

    k_spinlock_key_t key = k_spin_lock(&data->lock);

    while ((INBYTE(dev, LSR(dev)) & LSR_THRE) == 0)
    {
    }

    OUTBYTE(dev, THR(dev), c);

    k_spin_unlock(&data->lock, key);
}

static int sc16is7xx_err_check(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    int check = (INBYTE(dev, LSR(dev)) & LSR_EOB_MASK);

    k_spin_unlock(&data->lock, key);

    return check >> 1;
}

#if CONFIG_SC16IS7XX_INTERRUPT_DRIVEN

static int sc16is7xx_fifo_fill(const struct device *dev,
                               const uint8_t *tx_data,
                               int size)
{
    struct sc16is7xx_dev_data *data = dev->data;
    int i;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    for (i = 0; (i < size) && (i < data->fifo_size); i++)
    {
        OUTBYTE(dev, THR(dev), tx_data[i]);
    }

    k_spin_unlock(&data->lock, key);

    return i;
}

static int sc16is7xx_fifo_read(const struct device *dev, uint8_t *rx_data,
                               const int size)
{
    struct sc16is7xx_dev_data *data = dev->data;
    int i;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    for (i = 0; (i < size) && (INBYTE(dev, LSR(dev)) & LSR_RXRDY) != 0; i++)
    {
        rx_data[i] = INBYTE(dev, RDR(dev));
    }

    k_spin_unlock(&data->lock, key);

    return i;
}

static void sc16is7xx_irq_tx_enable(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

#if defined(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN) && defined(CONFIG_PM)
    struct sc16is7xx_dev_data *const dev_data = dev->data;

    if (!dev_data->tx_stream_on)
    {
        dev_data->tx_stream_on = true;
        uint8_t num_cpu_states;
        const struct pm_state_info *cpu_states;

        num_cpu_states = pm_state_cpu_get_all(0U, &cpu_states);

        for (uint8_t i = 0U; i < num_cpu_states; i++)
        {
            pm_policy_state_lock_get(cpu_states[i].state, PM_ALL_SUBSTATES);
        }
    }
#endif
    OUTBYTE(dev, IER(dev), INBYTE(dev, IER(dev)) | IER_TBE);

    k_spin_unlock(&data->lock, key);
}

static void sc16is7xx_irq_tx_disable(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    OUTBYTE(dev, IER(dev), INBYTE(dev, IER(dev)) & (~IER_TBE));

#if defined(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN) && defined(CONFIG_PM)
    struct sc16is7xx_dev_data *const dev_data = dev->data;

    if (dev_data->tx_stream_on)
    {
        dev_data->tx_stream_on = false;
        uint8_t num_cpu_states;
        const struct pm_state_info *cpu_states;

        num_cpu_states = pm_state_cpu_get_all(0U, &cpu_states);

        for (uint8_t i = 0U; i < num_cpu_states; i++)
        {
            pm_policy_state_lock_put(cpu_states[i].state, PM_ALL_SUBSTATES);
        }
    }
#endif
    k_spin_unlock(&data->lock, key);
}

static int sc16is7xx_irq_tx_ready(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    int ret = ((IIRC(dev) & IIR_ID) == IIR_THRE) ? 1 : 0;

    k_spin_unlock(&data->lock, key);

    return ret;
}

static int sc16is7xx_irq_tx_complete(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    int ret = ((INBYTE(dev, LSR(dev)) & (LSR_TEMT | LSR_THRE)) ==
               (LSR_TEMT | LSR_THRE))
                  ? 1
                  : 0;

    k_spin_unlock(&data->lock, key);

    return ret;
}

static void sc16is7xx_irq_rx_enable(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    OUTBYTE(dev, IER(dev), INBYTE(dev, IER(dev)) | IER_RXRDY);

    k_spin_unlock(&data->lock, key);
}

static void sc16is7xx_irq_rx_disable(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    OUTBYTE(dev, IER(dev), INBYTE(dev, IER(dev)) & (~IER_RXRDY));

    k_spin_unlock(&data->lock, key);
}

static int sc16is7xx_irq_rx_ready(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    int ret = ((IIRC(dev) & IIR_ID) == IIR_RBRF) ? 1 : 0;

    k_spin_unlock(&data->lock, key);

    return ret;
}

static void sc16is7xx_irq_err_enable(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    OUTBYTE(dev, IER(dev), INBYTE(dev, IER(dev)) | IER_LSR);

    k_spin_unlock(&data->lock, key);
}

static void sc16is7xx_irq_err_disable(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    OUTBYTE(dev, IER(dev), INBYTE(dev, IER(dev)) & (~IER_LSR));

    k_spin_unlock(&data->lock, key);
}

static int sc16is7xx_irq_is_pending(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    int ret = (!(IIRC(dev) & IIR_NIP)) ? 1 : 0;

    k_spin_unlock(&data->lock, key);

    return ret;
}

static int sc16is7xx_irq_update(const struct device *dev)
{
    struct sc16is7xx_dev_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    IIRC(dev) = INBYTE(dev, IIR(dev));

    k_spin_unlock(&data->lock, key);

    return 1;
}

static void sc16is7xx_irq_callback_set(const struct device *dev,
                                       uart_irq_callback_user_data_t cb,
                                       void *cb_data)
{
    struct sc16is7xx_dev_data *const dev_data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&dev_data->lock);

    dev_data->cb = cb;
    dev_data->cb_data = cb_data;

    k_spin_unlock(&dev_data->lock, key);
}

static void sc16is7xx_isr(const struct device *dev)
{
    struct sc16is7xx_dev_data *const dev_data = dev->data;

    if (dev_data->cb)
    {
        dev_data->cb(dev, dev_data->cb_data);
    }
}

#endif /* CONFIG_SC16IS7XX_INTERRUPT_DRIVEN */

#ifdef CONFIG_SC16IS7XX_LINE_CTRL

static int sc16is7xx_line_ctrl_set(const struct device *dev,
                                   uint32_t ctrl, uint32_t val)
{
    struct sc16is7xx_dev_data *data = dev->data;
    const struct sc16is7xx_device_config *const config = dev->config;
    uint32_t mcr, chg;
    k_spinlock_key_t key;

    switch (ctrl)
    {
    case UART_LINE_CTRL_BAUD_RATE:
        set_baud_rate(dev, val, config->sys_clk_freq);
        return 0;

    case UART_LINE_CTRL_RTS:
    case UART_LINE_CTRL_DTR:
        key = k_spin_lock(&data->lock);
        mcr = INBYTE(dev, MCR(dev));

        if (ctrl == UART_LINE_CTRL_RTS)
        {
            chg = MCR_RTS;
        }
        else
        {
            chg = MCR_DTR;
        }

        if (val)
        {
            mcr |= chg;
        }
        else
        {
            mcr &= ~(chg);
        }
        OUTBYTE(dev, MCR(dev), mcr);
        k_spin_unlock(&data->lock, key);
        return 0;
    }

    return -ENOTSUP;
}

#endif /* CONFIG_SC16IS7XX_LINE_CTRL */

static const struct uart_driver_api sc16is7xx_driver_api = {
    .poll_in = sc16is7xx_poll_in,
    .poll_out = sc16is7xx_poll_out,
    .err_check = sc16is7xx_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
    .configure = sc16is7xx_configure,
    .config_get = sc16is7xx_config_get,
#endif
#ifdef CONFIG_SC16IS7XX_INTERRUPT_DRIVEN
    .fifo_fill = sc16is7xx_fifo_fill,
    .fifo_read = sc16is7xx_fifo_read,
    .irq_tx_enable = sc16is7xx_irq_tx_enable,
    .irq_tx_disable = sc16is7xx_irq_tx_disable,
    .irq_tx_ready = sc16is7xx_irq_tx_ready,
    .irq_tx_complete = sc16is7xx_irq_tx_complete,
    .irq_rx_enable = sc16is7xx_irq_rx_enable,
    .irq_rx_disable = sc16is7xx_irq_rx_disable,
    .irq_rx_ready = sc16is7xx_irq_rx_ready,
    .irq_err_enable = sc16is7xx_irq_err_enable,
    .irq_err_disable = sc16is7xx_irq_err_disable,
    .irq_is_pending = sc16is7xx_irq_is_pending,
    .irq_update = sc16is7xx_irq_update,
    .irq_callback_set = sc16is7xx_irq_callback_set,
#endif
#ifdef CONFIG_SC16IS7XX_LINE_CTRL
    .line_ctrl_set = sc16is7xx_line_ctrl_set,
#endif
};

/* Helper macro to convert string token to enum */
#define SERIAL_MODE_ENUM(node_id)                                                                                                                  \
    (DT_ENUM_HAS_VALUE(node_id, serial_mode, rs485) ? SC16IS7XX_MODE_RS485 : DT_ENUM_HAS_VALUE(node_id, serial_mode, rs422) ? SC16IS7XX_MODE_RS422 \
                                                                                                                            : SC16IS7XX_MODE_RS232)

/* Device initialization macro for nested child binding */
#define SC16IS7XX_DEVICE_INIT(node_id)                                           \
    static const struct sc16is7xx_device_config sc16is7xx_dev_cfg_##node_id = {  \
        .port = DT_REG_ADDR(DT_PARENT(node_id)),                                 \
        .serport_num = DT_REG_ADDR(node_id),                                     \
        .bus = I2C_DT_SPEC_GET(DT_PARENT(node_id)),                              \
        .sys_clk_freq = DT_PROP(DT_PARENT(node_id), clock_frequency),            \
        .mode = SERIAL_MODE_ENUM(node_id),                                       \
        IF_ENABLED(CONFIG_SC16IS7XX_INTERRUPT_DRIVEN,                            \
                   (.int_gpio = GPIO_DT_SPEC_GET_OR(DT_PARENT(node_id),          \
                                                    interrupt_gpios, {}), ))};   \
    static struct sc16is7xx_dev_data sc16is7xx_dev_data_##node_id = {            \
        .uart_config.baudrate = DT_PROP_OR(node_id, current_speed, 9600),        \
        .uart_config.parity = DT_PROP_OR(node_id, parity, UART_CFG_PARITY_NONE), \
        .uart_config.stop_bits = DT_PROP_OR(node_id, stop_bits,                  \
                                            UART_CFG_STOP_BITS_1),               \
        .uart_config.data_bits = DT_PROP_OR(node_id, data_bits,                  \
                                            UART_CFG_DATA_BITS_8),               \
        .uart_config.flow_ctrl = COND_CODE_1(DT_PROP(node_id, hw_flow_control),  \
                                             (UART_CFG_FLOW_CTRL_RTS_CTS),       \
                                             (UART_CFG_FLOW_CTRL_NONE)),         \
    };                                                                           \
    DEVICE_DT_DEFINE(node_id,                                                    \
                     &sc16is7xx_init,                                            \
                     NULL,                                                       \
                     &sc16is7xx_dev_data_##node_id,                              \
                     &sc16is7xx_dev_cfg_##node_id,                               \
                     POST_KERNEL,                                                \
                     CONFIG_SERIAL_INIT_PRIORITY,                                \
                     &sc16is7xx_driver_api);

/* Initialize all serial child nodes under all sc16is7xx instances */
#define SC16IS7XX_INIT_CHILDREN(inst) \
    DT_FOREACH_CHILD_STATUS_OKAY(DT_DRV_INST(inst), SC16IS7XX_DEVICE_INIT)

/* Call for each instance of the parent device */
DT_INST_FOREACH_STATUS_OKAY(SC16IS7XX_INIT_CHILDREN)