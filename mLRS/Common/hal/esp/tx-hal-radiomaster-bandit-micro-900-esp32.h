//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// hal
//*******************************************************


//-------------------------------------------------------
// ESP32, ELRS RADIOMASTER BANDIT MICRO 900 TX
//-------------------------------------------------------
/*
  "serial_rx": 13,
  "serial_tx": 13,
  "radio_dio0": 22,
  "radio_dio1": 21,
  "radio_miso": 19,
  "radio_mosi": 23,
  "radio_nss": 4,
  "radio_rst": 5,
  "radio_sck": 18,
  "radio_dcdc": true,       ???
  "radio_rfo_hf": true,
  "power_txen": 33,
  "power_apc2": 26,
  "power_min": 3,
  "power_high": 6,
  "power_max": 6,
  "power_default": 3,
  "power_control": 3,
  "power_values": [168,148,128,90],
  "power_values2": [2,6,9,12],
  "use_backpack": true,
  "debug_backpack_baud": 460800,
  "debug_backpack_rx": 16,
  "debug_backpack_tx": 17,
  "backpack_boot": 32,
  "backpack_en": 25,
  "passthrough_baud": 230400,
  "led_red": 15,
  "led_red_invert": true,
  "misc_fan_en": 2,
  "screen_type": 1,
  "screen_sck": 12,
  "screen_sda": 14,
  "screen_reversed": true,
  "joystick": 39,
  "joystick_values": [3227,0,1961,2668,1290,4095]
*/


//#define DEVICE_HAS_JRPIN5
//#define DEVICE_HAS_IN
//#define DEVICE_HAS_SERIAL_OR_COM    // board has UART which is shared between Serial or Com, selected by e.g. a switch
//#define DEVICE_HAS_NO_SERIAL
#define DEVICE_HAS_NO_COM
#define DEVICE_HAS_NO_DEBUG

#define DEVICE_HAS_SINGLE_LED
//#define DEVICE_HAS_I2C_DISPLAY_ROT180
//#define DEVICE_HAS_FIVEWAY


//-- UARTS
// UARTB = serial port
// UARTC or USB = COM (CLI)
// UARTD = -
// UART  = JR bay pin5
// UARTE = in port, SBus or whatever
// UARTF = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                TX_SERIAL_BAUDRATE
#define UARTB_USE_TX_IO           IO_P17
#define UARTB_USE_RX_IO           IO_P16
#define UARTB_TXBUFSIZE           1024 // TX_SERIAL_TXBUFSIZE
#define UARTB_RXBUFSIZE           TX_SERIAL_RXBUFSIZE

#define UARTC_USE_SERIAL
#define UARTC_BAUD                115200
#define UARTC_TXBUFSIZE           0 // ?? // TX_COM_TXBUFSIZE
#define UARTC_RXBUFSIZE           TX_COM_RXBUFSIZE

#define UARTF_USE_SERIAL
#define UARTF_BAUD                115200
#define UARTF_TXBUFSIZE           0 // ?? // 512


//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 IO_P4
#define SPI_MISO                  IO_P19
#define SPI_MOSI                  IO_P23
#define SPI_SCK                   IO_P18
#define SPI_FREQUENCY             10000000L
#define SX_RESET                  IO_P5
#define SX_DIO0                   IO_P22
#define SX_TX_EN                  IO_P33
//#define SX_RX_EN                  IO_P?

#define SX_USE_RFO

IRQHANDLER(void SX_DIO_EXTI_IRQHandler(void);)

void sx_init_gpio(void)
{
    gpio_init(SX_DIO0, IO_MODE_INPUT_ANALOG);
    gpio_init(SX_TX_EN, IO_MODE_OUTPUT_PP_LOW);
    gpio_init(SX_RESET, IO_MODE_OUTPUT_PP_HIGH);
}

IRAM_ATTR void sx_amp_transmit(void)
{
//    gpio_low(SX_RX_EN);
    gpio_high(SX_TX_EN);
}

IRAM_ATTR void sx_amp_receive(void)
{
    gpio_low(SX_TX_EN);
//    gpio_high(SX_RX_EN);
}

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
//    gpio_init(BUTTON, IO_MODE_INPUT_PU);
}

IRAM_ATTR bool button_pressed(void)
{
    return false; // gpio_read_activelow(BUTTON) ? true : false;
}


//-- LEDs

#define LED_RED                   IO_P15

void leds_init(void)
{
    gpio_init(LED_RED, IO_MODE_OUTPUT_PP_LOW);
}

IRAM_ATTR void led_red_off(void) { gpio_low(LED_RED); }
IRAM_ATTR void led_red_on(void) { gpio_high(LED_RED); }
IRAM_ATTR void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- Display I2C

#define I2C_SDA_IO                IO_P14
#define I2C_SCL_IO                IO_P12
#define I2C_CLOCKSPEED            1000000L  // fix - rather too much, but helps with LQ, ESP32 max speed
#define I2C_BUFFER_SIZE           1024


//-- 5 Way Switch

#define FIVEWAY_ADC_IO            IO_P39
#define KEY_UP_THRESH             2800 // tom: 2839 // ow: 2966 
#define KEY_DOWN_THRESH           2200 // tom: 2191 // ow: 2282
#define KEY_LEFT_THRESH           1650 // tom: 1616 // ow: 1685
#define KEY_RIGHT_THRESH          3600 // tom: 3511 // ow: 3712
#define KEY_CENTER_THRESH         0

#if defined DEVICE_HAS_I2C_DISPLAY || defined DEVICE_HAS_I2C_DISPLAY_ROT180 || defined DEVICE_HAS_FIVEWAY

void fiveway_init(void) {} // no init needed to read an analog pin in Arduino

IRAM_ATTR uint16_t fiveway_adc_read(void)
{
    return analogRead(FIVEWAY_ADC_IO);
}

IRAM_ATTR uint8_t fiveway_read(void)
{
    uint16_t adc = analogRead(FIVEWAY_ADC_IO);
    if (adc < (KEY_CENTER_THRESH+200)) return (1 << KEY_CENTER); // 0
    if (adc > (KEY_LEFT_THRESH-200) && adc < (KEY_LEFT_THRESH+200)) return (1 << KEY_LEFT); 
    if (adc > (KEY_DOWN_THRESH-200) && adc < (KEY_DOWN_THRESH+200)) return (1 << KEY_DOWN);
    if (adc > (KEY_UP_THRESH-200) && adc < (KEY_UP_THRESH+200)) return (1 << KEY_UP);
    if (adc > (KEY_RIGHT_THRESH-300) && adc < (KEY_RIGHT_THRESH+300)) return (1 << KEY_RIGHT);
    return 0;
}
#endif


//-- Serial or Com Switch
// use com if FIVEWAY is DOWN during power up, else use serial
// FIVEWAY-DONW becomes bind button later on

#ifdef DEVICE_HAS_SERIAL_OR_COM
bool tx_ser_or_com_serial = true; // we use serial as default

void ser_or_com_init(void)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < 16; i++) {
        uint16_t adc = analogRead(FIVEWAY_ADC_IO);
        if (adc > (KEY_DOWN_THRESH-200) && adc < (KEY_DOWN_THRESH+200)) cnt++;
    }
    tx_ser_or_com_serial = !(cnt > 8);
}

IRAM_ATTR bool ser_or_com_serial(void)
{
    return tx_ser_or_com_serial;
}
#endif


//-- Cooling Fan

#define FAN_IO                    IO_P2

void fan_init(void)
{
    gpio_init(FAN_IO, IO_MODE_OUTPUT_PP_LOW);
    gpio_low(FAN_IO);
}

IRAM_ATTR void fan_set_power(int8_t power_dbm)
{
    if (power_dbm >= POWER_23_DBM) {
        gpio_high(FAN_IO);
    } else {
        gpio_low(FAN_IO);
    }
}


//-- POWER

#define POWER_GAIN_DBM            0 // 13 // gain of a PA stage if present
#define POWER_SX1276_MAX_DBM      SX1276_OUTPUT_POWER_MAX // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_MIN, .mW = INT8_MIN },
    { .dbm = POWER_20_DBM, .mW = 100 },
    { .dbm = POWER_24_DBM, .mW = 250 },
    { .dbm = POWER_27_DBM, .mW = 500 },
};
