//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// hal
//********************************************************

//-------------------------------------------------------
// DEVBOARD 2400 RX ESP32
//
// Uses a ESP32 Devboard with E28-2G4M27S
//
//-------------------------------------------------------

#define DEVICE_HAS_SINGLE_LED
#define DEVICE_HAS_SERIAL_OR_DEBUG

//-- Timers, Timing, EEPROM, and such stuff

#define DELAY_USE_DWT

#define SYSTICK_TIMESTEP          1000
#define SYSTICK_DELAY_MS(x)       (uint16_t)(((uint32_t)(x)*(uint32_t)1000)/SYSTICK_TIMESTEP)

#define EE_START_PAGE             0 // 128 kB flash, 2 kB page

#define MICROS_TIMx               TIM15

//-- UARTS
// UARTB = serial port
// UART = output port, SBus or whatever
// UARTC = debug port

#define UARTB_USE_SERIAL
#define UARTB_BAUD                  RX_SERIAL_BAUDRATE
#define UARTB_USE_TX
#define UARTB_TXBUFSIZE             RX_SERIAL_TXBUFSIZE // 1024 // 512
#define UARTB_USE_TX_ISR
#define UARTB_USE_RX
#define UARTB_RXBUFSIZE             RX_SERIAL_RXBUFSIZE // 1024 // 512

#define UARTC_USE_SERIAL
#define UARTC_BAUD                  115200

#define UART_USE_UART2_PA2PA3 // out pin
#define UART_BAUD                 100000 // SBus normal baud rate, is being set later anyhow
#define UART_USE_TX
#define UART_TXBUFSIZE            256 // 512
#define UART_USE_TX_ISR
//#define UART_USE_RX
//#define UART_RXBUFSIZE            512
#define OUT_UARTx                 USART2 // UART_UARTx is not known yet, so define by hand

//#define SWUART_USE_TIM15 // debug
#define SWUART_TX_IO              10
#define SWUART_BAUD               57600
#define SWUART_USE_TX
#define SWUART_TXBUFSIZE          512

//-- SX1: SX12xx & SPI

#define SPI_CS_IO                 27
#define HSPI_MISO                 33
#define HSPI_MOSI                 32
#define HSPI_SCLK                 25

#define SPI_FREQUENCY             12000000L

#define SX_RESET                  26
#define SX_DIO1                   35
#define SX_BUSY                   15

#define SX_TX_EN                  18
#define SX_RX_EN                  19

IRQHANDLER(void ICACHE_RAM_ATTR SX_DIO_EXTI_IRQHandler(void);)

#define     __IO    volatile             /*!< Defines 'read / write' permissions */

void sx_init_gpio(void)
{
    pinMode(SX_DIO1, INPUT);
    pinMode(SX_BUSY, INPUT);
    pinMode(SX_RESET, OUTPUT);    
    pinMode(SX_TX_EN, OUTPUT);
    pinMode(SX_RX_EN, OUTPUT);
    
    digitalWrite(SX_RESET, HIGH);
}

bool sx_busy_read(void)
{
    return (digitalRead(SX_BUSY) == HIGH) ? true : false;
}

void sx_amp_transmit(void) {
    digitalWrite(SX_RX_EN, LOW);
    digitalWrite(SX_TX_EN, HIGH);
 }

void sx_amp_receive(void) { 
    digitalWrite(SX_TX_EN, LOW);
    digitalWrite(SX_RX_EN, HIGH);
}

void sx_dio_enable_exti_isr(void)
{
    attachInterrupt(SX_DIO1, SX_DIO_EXTI_IRQHandler, RISING);
}

void sx_dio_init_exti_isroff(void)
{
    detachInterrupt(SX_DIO1);
}

void sx_dio_exti_isr_clearflag(void) {}


//-- Button

#define BUTTON                    0

void button_init(void)
{
    pinMode(BUTTON, INPUT_PULLUP);
}

bool button_pressed(void)
{
    return digitalRead(BUTTON) ? false : true;
}


//-- LEDs
#define LED_RED                   5

void leds_init(void)
{
    pinMode(LED_RED, OUTPUT);
    digitalWrite(LED_RED, HIGH);// LED_RED_OFF
}

void led_green_off(void) {  }
void led_green_on(void) {  }
void led_green_toggle(void) {  }

void led_red_off(void) { gpio_high(LED_RED); }
void led_red_on(void) { gpio_low(LED_RED); }
void led_red_toggle(void) { gpio_toggle(LED_RED); }


//-- POWER

#define POWER_GAIN_DBM            27 // gain of a PA stage if present
#define POWER_SX1280_MAX_DBM      SX1280_POWER_0_DBM  // maximum allowed sx power
#define POWER_USE_DEFAULT_RFPOWER_CALC

#define RFPOWER_DEFAULT           0 // index into rfpower_list array

const rfpower_t rfpower_list[] = {
    { .dbm = POWER_10_DBM, .mW = 10 },
    { .dbm = POWER_20_DBM, .mW = 100 },
};
