# sc16is7xx_zephyr_driver

Zephyr driver for the NXP I2C SC16IS7XX dual UART
Datasheet: https://www.nxp.com/docs/en/data-sheet/SC16IS752_SC16IS762.pdf

This is a cut and paste driver from the Zephyr 16550 driver (zephyr\drivers\serial\uart_ns16550.c) a few register definitions from the linux kernel (https://github.com/torvalds/linux/blob/master/drivers/tty/serial/sc16is7xx.c) with a few mods to drive the I2C from Zephyr.

It's a bit messy but hopefully will be enough to get you started, and you can contribute back to make it better for all of us.

This version has been modified by [robinlecouteur](https://github.com/robinlecouteur) to work with both channels of the sc16 as rs485.
Have added some external APIs like specifically for getting rs485 working with the custom wcm board

Here's an example of using x2 sc16 chips. They have been configured to 2 different i2c addresses. Refer to the datasheet for how these are set.
Setting the reg addres should be half the address set according to the docs since we use 7 bit addressing. The 8th bit is the read/write bit. So say the address is 0x90, divided by 2, that's 0x48.. REMEMBER hex addresses are HEX not decimal. hex 0x90 is 144 decimal, halved is 0x48 or 72 decimal.

Here's good article explaining how the addressing works: [ARTICLE LINK](https://www.totalphase.com/support/articles/200349176-7-bit-8-bit-and-10-bit-i2c-slave-addressing/)

The crystal can be either 1.8432 MHz or 3.072MHz, we used 1.8432 MHz so make sure you set that correctly

```cpp
&i2c0 {
	sc16is752_0: sc16is7xx@48 {
		compatible = "nxp,sc16is7xx";
		reg = <0x48>;  /* I2C address */
		clock-frequency = <1843200>;  /* 1.8432 MHz crystal */
		num-ports = <2>;
		#address-cells = <1>;
		#size-cells = <0>;


		/* Channel A */
		uart_ext0: serial@0 {
			compatible = "nxp,sc16is7xx-serial";
			reg = <0>;  /* Port A (channel 0) */
			current-speed = <9600>;
			serial-mode = "rs485";
			status = "okay";
		};

		/* Channel B */
		uart_ext1: serial@1 {
			compatible = "nxp,sc16is7xx-serial";
			reg = <1>;  /* Port B (channel 1) */
			current-speed = <9600>;
			serial-mode = "rs485";
			status = "okay";
		};
	};
	sc16is752_1: sc16is7xx@49 {
		compatible = "nxp,sc16is7xx";
		reg = <0x49>;  /* I2C address */
		clock-frequency = <1843200>;  /* 1.8432 MHz crystal */
		num-ports = <2>;
		#address-cells = <1>;
		#size-cells = <0>;


		/* Channel A  */
		uart_ext2: serial@0 {
			compatible = "nxp,sc16is7xx-serial";
			reg = <0>;  /* Port A (channel 0) */
			current-speed = <9600>;
			serial-mode = "rs485";
			status = "okay";
		};

		/* Channel B */
		uart_ext3: serial@1 {
			compatible = "nxp,sc16is7xx-serial";
			reg = <1>;  /* Port B (channel 1) */
			current-speed = <9600>;
			serial-mode = "rs485";
			status = "okay";
		};
	};

};
```

To use the uarts as normal uarts, use the polling mode functions like
uart_poll_out and uart_poll_in using the specific uart peripheral that we set up like uart_ext0

If you are using the chip with rs485 converters, in our particular case we manually use the external functions provided by this library.

To set the rs485 state (since we are half duplex) we use sc16is7xx_set_rs485_state.
Note that there are 3 states. Normally you would tie RE and DE together and connect them to the RTS for auto mode switching, but our rs485 converters only go into low power mode if the pins are set to disabled for both send and receive, so we control these manually using the sc16 GPIOs.This way allows is to have finer control over power consumtion. This may be revised in the future by having them tied to RTS, but have a switch on one of the pins attached to a single GPIO to allow switching to low power, but also use the automatic features of the SC16.

```c
	//
    typedef enum
    {
        SC16IS7XX_RS485_OFF, /* DE=LOW, RE=HIGH (disabled, low power) */
        SC16IS7XX_RS485_RX,  /* DE=LOW, RE=LOW  (receive mode) */
        SC16IS7XX_RS485_TX   /* DE=HIGH, RE=HIGH (transmit mode) */
    } sc16is7xx_rs485_state_
	///....

	// Pass in the uart peripheral of the channel we want to set, internally the sc16 driver finds what sc16 chip it's part of and sets pins for the channel that this uart is set up for. internally this is really just GPIO control
	sc16is7xx_set_rs485_state(uart_ext0, SC16IS7XX_RS485_TX)
```

We can also control any of the the GPIOs of the chip manually using sc16is7xx_gpio_control
In the WCM implementation, we use the GPIOs of the first sc16 chip to turn the 12v and 5v rails on to supply power to the rs485 devices.
Technically we can se the DE and RE pins as well, but that would be a bad idea since we set them using sc16is7xx_set_rs485_state.
Not sure the best way to abstract things, and a lot of the design choices of this driver version are tailored specifically to this particular hardware implementation. Figured the best thing was to just expose some custom functions and not worry too much. Someone else can adapt this driver further or abstract out the custom features into a specific rs485/modbus driver and keep this driver more "pure"

```c
// GPIO pin definitions defined in sc16is7xx.h....
#define SC16IS7XX_GPIO0 (1 << 0) /* Channel A DE */
#define SC16IS7XX_GPIO1 (1 << 1) /* Channel A RE */
#define SC16IS7XX_GPIO2 (1 << 2) /* Channel B DE */
#define SC16IS7XX_GPIO3 (1 << 3) /* Channel B RE */
#define SC16IS7XX_GPIO4 (1 << 4) /* Port A Power Enable */
#define SC16IS7XX_GPIO5 (1 << 5) /* Port B Power Enable */
#define SC16IS7XX_GPIO6 (1 << 6) /* 5V Power Enable */
#define SC16IS7XX_GPIO7 (1 << 7) /* 12V Power Enable */

// Named values in implementation
#define POWER_12V_EN SC16IS7XX_GPIO7
//...etc

// Pass in the uart device for EITHER channel of the sc16, this method will find the parent sc16 device
// Then set the bitmask as to what pins we are setting, then provide the value you want to set these pins to.
sc16is7xx_gpio_control(uart_ext0, POWER_12V_EN | POWER_5V_EN, POWER_12V_EN | POWER_5V_EN, 1 /*or 0x01*/);

```

THIS EXAMPLE IS FROM THE ORIGINAL CODEBASE, interrupt mode hasn't been tested since we do not have the interrupt pin connected to the NRF
Recommended to test this and make sure it actually still works

Then you can use the device as a normal UART like so, (add these functions to main.c or whatever)

```c
    #include "zephyr/drivers/uart.h"

    #define FEATHER_UART DT_NODELABEL(opito_uart)
    #if DT_NODE_HAS_STATUS(FEATHER_UART,okay)
    	const struct device *uart_dev = DEVICE_DT_GET(FEATHER_UART);
    #else
    	#error "uart2 device is disabled."
    #endif
    static uint8_t uart_buf[256];
    static const char *poll_data = "This is a POLL test.\r\n";



    void uart_cb(struct device *dev)
    {
        uart_irq_update(dev);
        int data_length = 0;
        if (uart_irq_rx_ready(dev)) {
    	data_length = uart_fifo_read(dev, uart_buf, sizeof(uart_buf));
    	uart_buf[data_length] = 0;
    		LOG_WRN("Rx from i2c uart %s", uart_buf);
        }
    }


    static bool initUART2(){

        if (uart_dev == NULL) {
    	LOG_WRN("Cannot bind %s\n", "opito_uart");
    	return false;
        }
        uart_irq_callback_set(uart_dev, uart_cb);
    	uart_irq_rx_enable(uart_dev);
        return true;
    }

    void uart2_start(void){
        	initUART2();
    	for (int i = 0; i < strlen(poll_data); i++) {
    		uart_poll_out(uart_dev, poll_data[i]);
    	}
    }
```
