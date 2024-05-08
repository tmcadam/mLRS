//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP UART
//********************************************************
#ifndef ESPLIB_UART_H
#define ESPLIB_UART_H


#ifndef ESPLIB_UART_ENUMS
#define ESPLIB_UART_ENUMS

typedef enum {
    XUART_PARITY_NO = 0,
    XUART_PARITY_EVEN,
    XUART_PARITY_ODD,
} UARTPARITYENUM;

typedef enum {
//    UART_STOPBIT_0_5 = 0, // not supported by ESP
    UART_STOPBIT_1 = 0,
    UART_STOPBIT_2,
} UARTSTOPBITENUM;

#endif


#ifdef UART_USE_SERIAL
  #define UART_SERIAL_NO       Serial
#elif defined UART_USE_SERIAL1
  #define UART_SERIAL_NO       Serial1
#elif defined UART_USE_SERIAL2
  #define UART_SERIAL_NO       Serial2
#elif defined UART_USE_HALFD_13
  #define UART_SERIAL_NO       Serial
#else
  #error UART_SERIAL_NO must be defined!
#endif

#ifndef UART_TXBUFSIZE
  #define UART_TXBUFSIZE       256 // MUST be 2^N
#endif
#ifndef UART_RXBUFSIZE
  #define UART_RXBUFSIZE       256 // MUST be 2^N
#endif

#if (defined UART_USE_TX_ISR) && (!defined UART_USE_TX)
  #error UART_USE_TX_ISR used without UART_USE_TX!
#endif
#if (defined UART_USE_RXERRORCOUNT) && (!defined UART_USE_RX)
  #error UART_USE_RXERRORCOUNT used without UART_USE_RX!
#endif

#ifdef UART_USE_TX_ISR
  #ifndef UART_TXBUFSIZE
    #define UART_TXBUFSIZE      256 // MUST be 2^N
  #endif
  #if UART_TXBUFSIZE < 2
    #error UART_TXBUFSIZE must be larger than 1 !
  #elif ((UART_TXBUFSIZE & (UART_TXBUFSIZE-1)) != 0)
    #error UART_TXBUFSIZE must be a power of 2 !
  #endif

  #define UART_TXBUFSIZEMASK  (UART_TXBUFSIZE-1)

  volatile char uart_txbuf[UART_TXBUFSIZE];
  volatile uint16_t uart_txwritepos; // pos at which the last byte was stored
  volatile uint16_t uart_txreadpos; // pos at which the next byte is to be fetched
#endif

#ifdef UART_USE_RX
  #ifndef UART_RXBUFSIZE
    #define UART_RXBUFSIZE      256 //128 //MUST be 2^N
  #endif
  #if UART_RXBUFSIZE < 2
    #error UART_RXBUFSIZE must be larger than 1 !
  #elif ((UART_RXBUFSIZE & (UART_RXBUFSIZE-1)) != 0)
    #error UART_RXBUFSIZE must be a power of 2 !
  #endif

  #define UART_RXBUFSIZEMASK  (UART_RXBUFSIZE-1)

  volatile char uart_rxbuf[UART_RXBUFSIZE];
  volatile uint16_t uart_rxwritepos; // pos at which the last byte was stored
  volatile uint16_t uart_rxreadpos; // pos at which the next byte is to be fetched

  #ifdef UART_USE_RXERRORCOUNT
  volatile uint32_t uart_errorcnt_rxnoise;
  volatile uint32_t uart_errorcnt_rxframe;
  volatile uint32_t uart_errorcnt_rxoverrun;
  #endif
#endif


IRAM_ATTR void uart_putbuf(uint8_t* buf, uint16_t len)
{
    UART_SERIAL_NO.write((uint8_t*)buf, len);
}


IRAM_ATTR char uart_getc(void)
{
    return (char)UART_SERIAL_NO.read();
}


IRAM_ATTR void uart_rx_flush(void)
{
    while (UART_SERIAL_NO.available() > 0) UART_SERIAL_NO.read();
}


IRAM_ATTR void uart_tx_flush(void)
{
    UART_SERIAL_NO.flush();
}


IRAM_ATTR uint16_t uart_rx_bytesavailable(void)
{
    return (UART_SERIAL_NO.available() > 0) ? UART_SERIAL_NO.available() : 0;
}


IRAM_ATTR uint16_t uart_rx_available(void)
{
    return (UART_SERIAL_NO.available() > 0) ? 1 : 0;
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------
// Note: ESP32 has a hardware fifo for tx, which is 128 bytes in size. However, MAVLink messages
// can be larger than this, and data would thus be lost when put only into the fifo. It is therefore
// crucial to set a Tx buffer size of sufficient size. setTxBufferSize() is not available for ESP82xx.

void _uart_initit(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
#ifdef ESP32
    UART_SERIAL_NO.setTxBufferSize(UART_TXBUFSIZE);
    UART_SERIAL_NO.setRxBufferSize(UART_RXBUFSIZE);

    uint32_t config = SERIAL_8N1;
    switch (parity) {
        case XUART_PARITY_NO:
            switch (stopbits) {
                case UART_STOPBIT_1: config = SERIAL_8N1; break;
                case UART_STOPBIT_2: config = SERIAL_8N2; break;
            }
            break;
        case XUART_PARITY_EVEN:
            switch (stopbits) {
                case UART_STOPBIT_1: config = SERIAL_8E1; break;
                case UART_STOPBIT_2: config = SERIAL_8E2; break;
            }
            break;
        case XUART_PARITY_ODD:
            switch (stopbits) {
                case UART_STOPBIT_1: config = SERIAL_8O1; break;
                case UART_STOPBIT_2: config = SERIAL_8O2; break;
            }
            break;
    }
#if defined UART_USE_TX_IO || defined UART_USE_RX_IO // both need to be defined
    UART_SERIAL_NO.begin(baud, config, UART_USE_RX_IO, UART_USE_TX_IO);
#else
    UART_SERIAL_NO.begin(baud, config);
#endif

    UART_SERIAL_NO.setRxFIFOFull(8);  // > 57600 baud sets to 120 which is too much, buffer only 127 bytes
    UART_SERIAL_NO.setRxTimeout(1);   // wait for 1 symbol (~11 bits) to trigger Rx ISR, default 2

#elif defined ESP8266
    UART_SERIAL_NO.setRxBufferSize(UART_RXBUFSIZE);
    UART_SERIAL_NO.begin(baud);
#endif
}


void uart_setbaudrate(uint32_t baud)
{
    UART_SERIAL_NO.end();
    _uart_initit(baud, XUART_PARITY_NO, UART_STOPBIT_1);
}


void uart_setprotocol(uint32_t baud, UARTPARITYENUM parity, UARTSTOPBITENUM stopbits)
{
    UART_SERIAL_NO.end();
    _uart_initit(baud, parity, stopbits);
}


void uart_init(void)
{
    UART_SERIAL_NO.end();
    _uart_initit(UART_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uart_init_isroff(void)
{
    UART_SERIAL_NO.end();
    _uart_initit(UART_BAUD, XUART_PARITY_NO, UART_STOPBIT_1);
}

void uart_rx_enableisr(FunctionalState flag)
{
// #ifdef UART_USE_RX
//   if (flag == ENABLE) {
//     // Enable Receive Data register not empty interrupt
// #if defined STM32F1
//     LL_USART_ClearFlag_RXNE(UART_UARTx);
// #endif
//     LL_USART_ReceiveData8(UART_UARTx); // read DR to clear RXNE and error flags
//     LL_USART_EnableIT_RXNE(UART_UARTx);
//   } else {
//     LL_USART_DisableIT_RXNE(UART_UARTx);
// #if defined STM32F1
//     LL_USART_ClearFlag_RXNE(UART_UARTx);
// #endif
//   }
// #endif
}



#endif // ESPLIB_UART_H
