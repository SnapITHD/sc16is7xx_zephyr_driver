#ifndef ZEPHYR_INCLUDE_DRIVERS_SERIAL_SC16IS7XX_H_
#define ZEPHYR_INCLUDE_DRIVERS_SERIAL_SC16IS7XX_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief RS-485 transceiver states
     */
    typedef enum
    {
        SC16IS7XX_RS485_OFF, /* DE=LOW, RE=HIGH (disabled, low power) */
        SC16IS7XX_RS485_RX,  /* DE=LOW, RE=LOW  (receive mode) */
        SC16IS7XX_RS485_TX   /* DE=HIGH, RE=HIGH (transmit mode) */
    } sc16is7xx_rs485_state_t;

    /**
     * @brief Set RS-485 transceiver state
     * When switching to RX, automatically waits for TX buffer to empty
     */
    void sc16is7xx_set_rs485_state(const struct device *dev, sc16is7xx_rs485_state_t state);

    /**
     * @brief Sleep briefly if there has been a recent transaction on this uart
     *
     * @param dev UART device
     */
    void sc16is7xx_post_transaction_sleep(const struct device *dev);

    /**
     * @brief Control GPIO pins directly
     * @param dev
     * @param gpio_mask Which GPIO bits to modify (use GPIO pin defines)
     * @param gpio_value Value to set for those bits
     *
     * Example: sc16is7xx_gpio_control(dev, POWER_12V_EN | POWER_5V_EN, POWER_12V_EN | POWER_5V_EN);
     *          This sets GPIO6 and GPIO7 HIGH
     */
    void sc16is7xx_gpio_control(const struct device *dev, uint8_t gpio_mask, uint8_t gpio_value);

    /**
     * @brief Write data to TX FIFO (fills up to 64 bytes)
     * @param data Buffer to send
     * @param len Number of bytes to send
     * @return Number of bytes written to FIFO
     */
    int sc16is7xx_write_fifo(const struct device *dev, const uint8_t *data, size_t len);

    /**
     * @brief Check if TX buffer is empty
     * @return true if transmitter is completely empty
     */
    bool sc16is7xx_tx_empty(const struct device *dev);

    /**
     * @brief Read available data from RX FIFO
     * @param buffer Buffer to store received data
     * @param max_len Maximum bytes to read
     * @return Number of bytes read
     */
    int sc16is7xx_read_fifo(const struct device *dev, uint8_t *buffer, size_t max_len);

/* GPIO pin definitions - matches your hardware */
#define SC16IS7XX_GPIO0 (1 << 0) /* Channel A DE */
#define SC16IS7XX_GPIO1 (1 << 1) /* Channel A RE */
#define SC16IS7XX_GPIO2 (1 << 2) /* Channel B DE */
#define SC16IS7XX_GPIO3 (1 << 3) /* Channel B RE */
#define SC16IS7XX_GPIO4 (1 << 4) /* Port A Power Enable */
#define SC16IS7XX_GPIO5 (1 << 5) /* Port B Power Enable */
#define SC16IS7XX_GPIO6 (1 << 6) /* 5V Power Enable */
#define SC16IS7XX_GPIO7 (1 << 7) /* 12V Power Enable */

#ifdef __cplusplus
}
#endif

#endif