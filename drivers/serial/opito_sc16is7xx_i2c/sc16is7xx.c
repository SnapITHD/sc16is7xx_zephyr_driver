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
 * Configured for RS-485 operation with automatic direction control
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

LOG_MODULE_REGISTER(sc16is7xx, LOG_LEVEL_DBG);

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
/* Channel A (MODBUS Port 1) */
#define CHANNEL_A_TX_EN (1 << 0) // GPIO0 - Driver 1 enable (active HIGH to transmit)
#define CHANNEL_A_RX_EN (1 << 1) // GPIO1 - Receiver 1 enable (active LOW to receive)

/* Channel B (MODBUS Port 2) */
#define CHANNEL_B_TX_EN (1 << 2) // GPIO2 - Driver 2 enable (active HIGH to transmit)
#define CHANNEL_B_RX_EN (1 << 3) // GPIO3 - Receiver 2 enable (active LOW to receive)

/* Power control pins (GPIO4-7) - Not used for serial mode configuration */
#define RS485_PORT1_PWR_EN (1 << 4) // GPIO4 - Enable power for sensor port 1
#define RS485_PORT2_PWR_EN (1 << 5) // GPIO5 - Enable power for sensor port 2
#define POWER_5V_EN (1 << 5)        // GPIO5 - 5V_MODBUS_ON_OFF
#define POWER_12V_EN (1 << 6)       // GPIO6 - 12V_MODBUS_ON_OFF
#define HW_RESET_CTRL (1 << 7)      // GPIO7 - Hardware reset control

/* Masks for transceiver control only */
#define CHANNEL_A_MASK (CHANNEL_A_TX_EN | CHANNEL_A_RX_EN)
#define CHANNEL_B_MASK (CHANNEL_B_TX_EN | CHANNEL_B_RX_EN)

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
static void sc16is7xx_isr(const struct device *dev);
static int sc16is7xx_irq_update(const struct device *dev);

#define INBYTE(dev, x) sc16is17xx_reg_read(dev, x)
#define OUTBYTE(dev, x, d) sc16is17xx_reg_write(dev, x, d)

/* Serial interface modes */
enum sc16is7xx_serial_mode
{
    SC16IS7XX_MODE_RS232 = 0, /* Standard RS-232 */
    SC16IS7XX_MODE_RS485 = 1, /* RS-485 with auto direction control */
    SC16IS7XX_MODE_RS422 = 2, /* RS-422 (future support) */
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

static void set_reset(const struct device *dev)
{
    OUTBYTE(dev, IOCONTROL(dev), 0x08);
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

        OUTBYTE(dev, LCR(dev), LCR_EFR_ACCESS);
        OUTBYTE(dev, EFR(dev), 0x00);

        OUTBYTE(dev, LCR(dev), lcr_cache);

        dev_data->uart_config.baudrate = baud_rate;
    }
}

/**
 * @brief Configure UART for RS-232, RS-485, or RS-422 operation based on device tree
 *
 * RS-232 mode: Basic UART operation with RTS/CTS for flow control
 * RS-485 mode: Automatic direction control via RTS pin and GPIO transceiver control
 * RS-422 mode: Full-duplex differential signaling (future support)
 */
static void configure_uart_mode(const struct device *dev)
{
    const struct sc16is7xx_device_config *const dev_cfg = dev->config;
    uint8_t gpio_state = 0;
    uint8_t efcr = 0;
    uint8_t mcr = 0;

    /* Read current EFCR register */
    efcr = INBYTE(dev, EFCR(dev));

    switch (dev_cfg->mode)
    {
    case SC16IS7XX_MODE_RS485:
        /* ===== RS-485 MODE CONFIGURATION ===== */
        LOG_INF("Configuring channel %d for RS-485 mode", dev_cfg->serport_num);

        /* Set all GPIO pins as outputs for transceiver control */
        OUTBYTE(dev, IOCONTROL(dev), 0x00);
        OUTBYTE(dev, IODIR(dev), 0xFF);

        /* Read current GPIO state */
        gpio_state = INBYTE(dev, IOSTATE(dev));
        LOG_DBG("Initial GPIO state = 0x%02X", gpio_state);

        if (dev_cfg->serport_num == 0) /* Channel A (MODBUS Port 1) */
        {
            /* Clear transceiver control bits for Channel A */
            gpio_state &= ~CHANNEL_A_MASK;

            /* TX disabled (GPIO0 = 0), RX enabled (GPIO1 = 0, active LOW) */
            gpio_state &= ~CHANNEL_A_TX_EN;
            gpio_state &= ~CHANNEL_A_RX_EN;

            OUTBYTE(dev, IOSTATE(dev), gpio_state);

            LOG_INF("Channel A RS-485: GPIO=0x%02X", INBYTE(dev, IOSTATE(dev)));
        }
        else if (dev_cfg->serport_num == 1) /* Channel B (MODBUS Port 2) */
        {
            /* Clear transceiver control bits for Channel B */
            gpio_state &= ~CHANNEL_B_MASK;

            /* TX disabled (GPIO2 = 0), RX enabled (GPIO3 = 0, active LOW) */
            gpio_state &= ~CHANNEL_B_TX_EN;
            gpio_state &= ~CHANNEL_B_RX_EN;

            OUTBYTE(dev, IOSTATE(dev), gpio_state);

            LOG_INF("Channel B RS-485: GPIO=0x%02X", INBYTE(dev, IOSTATE(dev)));
        }

        /* Enable RS-485 automatic direction control via EFCR */
        efcr |= EFCR_AUTO_RS485_DIR; /* Enable auto direction control */
        efcr |= EFCR_RTS_INVERT;     /* RTS=1 during TX (for DE pin) */
        OUTBYTE(dev, EFCR(dev), efcr);

        LOG_INF("Channel %d RS-485 configured: EFCR=0x%02X",
                dev_cfg->serport_num, efcr);
        break;

    case SC16IS7XX_MODE_RS422:
        /* ===== RS-422 MODE CONFIGURATION ===== */
        LOG_INF("Configuring channel %d for RS-422 mode", dev_cfg->serport_num);

        /* RS-422 is full-duplex, so no direction control needed */
        /* Clear RS-485 bits */
        efcr &= ~EFCR_AUTO_RS485_DIR;
        efcr &= ~EFCR_RTS_INVERT;
        efcr &= ~EFCR_9BIT_MODE_BIT;

        OUTBYTE(dev, EFCR(dev), efcr);

        /* Configure GPIO for RS-422 transceiver enables if needed */
        /* RS-422 transceivers typically have separate TX/RX enable pins */
        OUTBYTE(dev, IOCONTROL(dev), 0x00);
        OUTBYTE(dev, IODIR(dev), 0xFF);

        gpio_state = INBYTE(dev, IOSTATE(dev));

        if (dev_cfg->serport_num == 0) /* Channel A */
        {
            /* For RS-422: Enable both TX and RX transceivers simultaneously */
            gpio_state |= CHANNEL_A_TX_EN;  /* TX enable HIGH */
            gpio_state &= ~CHANNEL_A_RX_EN; /* RX enable LOW (active low) */
            OUTBYTE(dev, IOSTATE(dev), gpio_state);

            LOG_INF("Channel A RS-422: GPIO=0x%02X", INBYTE(dev, IOSTATE(dev)));
        }
        else if (dev_cfg->serport_num == 1) /* Channel B */
        {
            gpio_state |= CHANNEL_B_TX_EN;  /* TX enable HIGH */
            gpio_state &= ~CHANNEL_B_RX_EN; /* RX enable LOW (active low) */
            OUTBYTE(dev, IOSTATE(dev), gpio_state);

            LOG_INF("Channel B RS-422: GPIO=0x%02X", INBYTE(dev, IOSTATE(dev)));
        }

        /* Set RTS/DTR active for good measure */
        mcr = INBYTE(dev, MCR(dev));
        mcr |= MCR_RTS_BIT | MCR_DTR_BIT;
        OUTBYTE(dev, MCR(dev), mcr);

        LOG_INF("Channel %d RS-422 configured: EFCR=0x%02X MCR=0x%02X",
                dev_cfg->serport_num, efcr, mcr);
        break;

    case SC16IS7XX_MODE_RS232:
    default:
        /* ===== RS-232 MODE CONFIGURATION ===== */
        LOG_INF("Configuring channel %d for RS-232 mode", dev_cfg->serport_num);

        /* Clear RS-485 bits to ensure normal RS-232 mode */
        efcr &= ~EFCR_AUTO_RS485_DIR; /* Disable auto RS-485 direction */
        efcr &= ~EFCR_RTS_INVERT;     /* Normal RTS polarity */
        efcr &= ~EFCR_9BIT_MODE_BIT;  /* Disable 9-bit/multidrop mode */

        OUTBYTE(dev, EFCR(dev), efcr);

        /* Set RTS and DTR to active (LOW) for standard RS-232 */
        mcr = INBYTE(dev, MCR(dev));
        mcr |= MCR_RTS_BIT | MCR_DTR_BIT; /* Assert RTS and DTR */
        OUTBYTE(dev, MCR(dev), mcr);

        LOG_INF("Channel %d RS-232 configured: EFCR=0x%02X MCR=0x%02X",
                dev_cfg->serport_num, efcr, mcr);
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

    /* Configure transceiver */
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
        goto out;
    }

    dev_data->uart_config = *cfg;

    OUTBYTE(dev, LCR(dev),
            uart_cfg.data_bits | uart_cfg.stop_bits | uart_cfg.parity);

    // mcr = MCR_RTS_BIT | MCR_DTR_BIT;
    if (cfg->flow_ctrl == UART_CFG_FLOW_CTRL_RTS_CTS)
    {
        /* Flow control if needed */
    }

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
    pclk = dev_cfg->sys_clk_freq;
    set_baud_rate(dev, cfg->baudrate, pclk);

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

    ret = sc16is17xx_reg_write(dev, SPR(dev), 0xAB);
    if (ret < 0)
    {
        LOG_ERR("Failed to write SPR register!");
        return ret;
    }

    uint8_t ping_req = sc16is17xx_reg_read(dev, SPR(dev));
    if (ping_req == 0xAB)
    {
        LOG_INF("SC16IS7XX channel %d communication OK", config->serport_num);
    }
    else
    {
        LOG_ERR("SC16IS7XX channel %d communication failed (expected 0xAB, got 0x%02X)",
                config->serport_num, ping_req);
        return -EIO;
    }

    ret = sc16is7xx_configure(dev, &data->uart_config);
    if (ret != 0)
    {
        LOG_ERR("sc16is7xx_configure failed: ret=[%d]", ret); // ← Add this
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
        LOG_ERR("sc16is7xx[0x%X]: failed to configure interrupt pin %d (%d)",
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