//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************

/*
  Info on DIP switches
   
  1,2 on:   update firmware on main ESP32, USB is connected to UO 
  3,4 on:   normal operation mode, USB is inoperational, UO connected to ESP8285
  5,6,7 on: update firmware on ESP8285, USB is connected to ESP8285's uart

  Flashing ESP8285:
  - Board: Generic ESP8266 Module
  - Reset Method: dtr (aka modemcu)
*/

//-------------------------------------------------------
// ESP32, ELRS BETAFPV MICRO 1W 2400 TX
//-------------------------------------------------------
/*
    Pin Defs
    "serial_rx": 13
    "serial_tx": 13
    "radio_busy": 21
    "radio_dio1": 4
    "radio_miso": 19
    "radio_mosi": 23
    "radio_nss": 5
    "radio_rst": 14
    "radio_sck": 18
    "power_rxen": 27
    "power_txen": 26
    "power_lna_gain": 12
    "power_min": 1
    "power_high": 6
    "power_max": 6
    "power_default": 2
    "power_control": 0
    "power_values": [-18,-15,-12,-7,-4,2]
    "joystick": 25
    "joystick_values": [2839,2191,1616,3511,0,4095]
    "led_rgb": 16
    "led_rgb_isgrb": true
    "screen_sck": 32
    "screen_sda": 22
    "screen_type": 1
    "screen_reversed": 1
    "use_backpack": true
    "debug_backpack_baud": 460800
    "debug_backpack_rx": 3
    "debug_backpack_tx": 1
    "misc_fan_en": 17
*/

#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN //for some reason sbus inv blocks the 5 way button
#define DEVICE_HAS_SERIAL_OR_COM // board has UART which is shared between Serial or Com, selected by e.g. a switch
//#define DEVICE_HAS_NO_SERIAL
//#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_NO_DEBUG
//#define DEVICE_HAS_SERIAL_OR_DEBUG

//-- UARTS
// UARTB = serial port
// UARTC or USB = COM (CLI)
// UARTD = serial2 BT/ESP port
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL // serial, is on P1/P3
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_TXBUFSIZE           1024 // TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_SERIAL // com USB/CLI, is on P1/P3
#define UARTC_BAUD                115200
#define UARTC_TXBUFSIZE           0 // ?? // TX_COM_TXBUFSIZE
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UART_USE_SERIAL2         // Make sure not in use with other UARTs
#define UART_USE_HALFD            // JR pin5, MBridge
#define UART_HALFD_PIN            13
#define UART_BAUD                 400000
#define UART_USE_TX
#define UART_TXBUFSIZE            256
#define UART_USE_TX_ISR
#define UART_USE_RX
#define UART_RXBUFSIZE            512

// #define UARTE_USE_SERIAL1 // in port is on P13
// #define UARTE_BAUD                100000 // SBus normal baud rate, is being set later anyhow
// #define UARTE_USE_TX_IO           -1 // no Tx pin needed
// #define UARTE_USE_RX_IO           13
// #define UARTE_TXBUFSIZE           0 // not used
// #define UARTE_RXBUFSIZE           512

#define UARTF_USE_SERIAL // debug
#define UARTF_BAUD                115200
#define UARTF_TXBUFSIZE           0 // ?? // 512


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P18
#define SPI_FREQUENCY             10000000L
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P27
#define SPI_SCK                   IO_P5
#define SX_RESET                  IO_P14
#define SX_DIO0                   IO_P26

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO0, IO_MODE_INPUT_ANALOG);


    
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
    gpio_init(IO_P23, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(IO_P22, IO_MODE_INPUT_PU);

    pinMode(IO_P33, OUTPUT);
    gpio_high(IO_P33); 

    pinMode(IO_P32, OUTPUT);
    gpio_high(IO_P32); 
}

IRAM_ATTR void sx_amp_transmit(void){}

IRAM_ATTR void sx_amp_receive(void){}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO0, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO0);
}

void sx_dio_exti_isr_clearflag(void) {}


//-- Button

#define BUTTON                    IO_P0

void button_init(void)
{
    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return gpio_read_activelow(BUTTON) ? true : false;
}


//-- LEDs

#define LED_RED                   IO_P2

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void led_red_off(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial
// FIVEWAY-DONW becomes bind button later on

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
    // could read a pin here
    tx_ser_or_com_serial = gpio_read_activelow(IO_P22) ? true : false;
}

IRAM_ATTR bool ser_or_com_serial(void)
{
    return tx_ser_or_com_serial;
}
#endif


//-- POWER

#define POWER_GAIN_DBM            0 // gain of a PA stage if present
#define POWER_SX1276_MAX_DBM      SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           1 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_0_DBM, .mW = 1 },
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_17_DBM, .mW = 50 },
};
