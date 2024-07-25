//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
// OlliW @ www.olliw.eu
//*******************************************************
// JR Pin5 Interface Header
//********************************************************
//
// Documentation
//
// 1. Methods which require external hardware, like xor and buffer chips
//   these are essentially used for F103 chips
//
//   #define JRPIN5_TX_XOR  : a xor is in the Tx line to invert the Tx signal
//
//   #define JRPIN5_RX_XOR  : a xor is in the Rx line to invert the Rx signal
//
//   #define JRPIN5_TX_OE   : Tx & Rx inverter with Tx buffer method
//
// 2. Methods which use the swap and invert capability of more modern UART peripherals
//   but need an external Schottky diode
//
//   #define JRPIN5_RX_TX_INVERT_INTERNAL
//     internal peripheral inverter method, needs a diode from Tx to Rx
//     the jrpin5 signal is on the Rx pin
//
//   #define JRPIN5_RX_TX_INVERT_SWAP_INTERNAL
//     internal peripheral inverter method with Tx<->Rx swap, needs a diode from Rx to Tx
//     the jrpin5 signal is on the Tx pin
//
// 3. Methods which use the swap and invert capability of more modern UART peripherals
//   but avoid the need of an external Schottky diode
//   they essentially supersede the other options
//
//   #define JRPIN5_FULL_INTERNAL_ON_TX     : jrpin5 signal is on the Tx pin
//
//   #define JRPIN5_FULL_INTERNAL_ON_RX     : jrpin5 signal is on the Rx pin
//
//   #define JRPIN5_FULL_INTERNAL_ON_RX_TX  : in cases there Rx & Tx pins are electrically connected
//
#ifndef JRPIN5_INTERFACE_H
#define JRPIN5_INTERFACE_H
#pragma once


#include "../Common/hal/hal.h" // not needed but helps editor to get defines correct LOL


extern volatile uint32_t millis32(void);


//-------------------------------------------------------
// Interface Implementation
// the uart used for JR pin5 must be UART_UARTx

void uart_rx_callback_dummy(uint8_t c) {}
void uart_tc_callback_dummy(void) {}

void (*uart_rx_callback_ptr)(uint8_t) = &uart_rx_callback_dummy;
void (*uart_tc_callback_ptr)(void) = &uart_tc_callback_dummy;

#define UART_RX_CALLBACK_FULL(c)    (*uart_rx_callback_ptr)(c)
#define UART_TC_CALLBACK()          (*uart_tc_callback_ptr)()

#ifdef ESP32
#include "../Common/esp-lib/esp-uart.h"
#else
#include "../modules/stm32ll-lib/src/stdstm32-uart.h"
#endif


// not available in stdstm32-uart.h, used for half-duplex mode
void uart_tx_putc_totxbuf(char c)
{
    uint16_t next = (uart_txwritepos + 1) & UART_TXBUFSIZEMASK;
    if (uart_txreadpos != next) { // fifo not full //this is isr safe, works also if readpos has changed in the meanwhile
        uart_txbuf[next] = c;
        uart_txwritepos = next;
    }
    //UART_SERIAL_NO.write(c);
}


// not available in stdstm32-uart.h, used for half-duplex mode
void uart_tx_start(void)
{
    //Serial.println("TX START");
    UART_SERIAL_NO.write((uint8_t*)uart_txbuf, uart_txwritepos + 1);
}


class tPin5BridgeBase
{
  public:
    void Init(void);

    // telemetry handling
    bool telemetry_start_next_tick;
    uint16_t telemetry_state;

    void TelemetryStart(void);

    // interface to the uart hardware peripheral used for the bridge, called in isr context
    void pin5_tx_start(void) { uart_tx_start(); }
    void pin5_putc(char c) { uart_tx_putc_totxbuf(c); }

    // for in-isr processing
    void pin5_tx_enable(bool enable_flag);
    virtual void parse_nextchar(uint8_t c) = 0;
    virtual bool transmit_start(void) = 0; // returns true if transmission should be started

    // actual isr functions
    void uart_rx_callback(uint8_t c);
    void uart_tc_callback(void);

    // parser
    typedef enum {
        STATE_IDLE = 0,

        // mBridge receive states
        STATE_RECEIVE_MBRIDGE_STX2,
        STATE_RECEIVE_MBRIDGE_LEN,
        STATE_RECEIVE_MBRIDGE_SERIALPACKET,
        STATE_RECEIVE_MBRIDGE_CHANNELPACKET,
        STATE_RECEIVE_MBRIDGE_COMMANDPACKET,

        // crsf receive states
        STATE_RECEIVE_CRSF_LEN,
        STATE_RECEIVE_CRSF_PAYLOAD,
        STATE_RECEIVE_CRSF_CRC,

        // transmit states, used by all
        STATE_TRANSMIT_START,
        STATE_TRANSMITING,
    } STATE_ENUM;

    // not used in this class, but required by the children, so just add them here
    // no need for volatile since used only in isr context
    uint8_t state;
    uint8_t len;
    uint8_t cnt;
    uint16_t tlast_us;

    // check and rescue
    // the FRM303 can get stuck, whatever we tried, so brutal rescue
    // can't hurt generally as safety net
    uint32_t nottransmiting_tlast_ms;
    void CheckAndRescue(void);
};


void tPin5BridgeBase::Init(void)
{
    state = STATE_IDLE;
    len = 0;
    cnt = 0;
    tlast_us = 0;

    telemetry_start_next_tick = false;
    telemetry_state = 0;

    nottransmiting_tlast_ms = 0;
    uart_init_isroff();
    uart_halfd_init();
    pin5_tx_enable(false); // also enables rx isr
}


void tPin5BridgeBase::TelemetryStart(void)
{
    telemetry_start_next_tick = true;
}


//-------------------------------------------------------
// Interface to the uart hardware peripheral used for the bridge
// called in isr context

void tPin5BridgeBase::pin5_tx_enable(bool enable_flag)
{
    if (enable_flag) {
        uart_rx_enableisr(DISABLE);
        uart_halfd_enable_tx();
    } else {
        uart_halfd_enable_rx();
        uart_rx_enableisr(ENABLE);
    }
}


// we do not add a delay here before we transmit
// the logic analyzer shows this gives a 30-35 us gap nevertheless, which is perfect

void tPin5BridgeBase::uart_rx_callback(uint8_t c)
{
    parse_nextchar(c);
    
    if (state < STATE_TRANSMIT_START) return; // we are in receiving

    if (state != STATE_TRANSMIT_START) { // we are in transmitting, should not happen! (does appear to not happen)
        state = STATE_IDLE;
        return;
    }

    if (transmit_start()) { // check if a transmission waits, put it into buf and return true to start
        pin5_tx_enable(true);
        state = STATE_TRANSMITING;
        pin5_tx_start();
    } else {
        state = STATE_IDLE;
    }
}


void tPin5BridgeBase::uart_tc_callback(void)
{
    pin5_tx_enable(false); // switches on rx
    state = STATE_IDLE;
}


//-------------------------------------------------------
// Check and rescue
// a good place to call it could be ChannelsUpdated()
// Note: For the FRM303 it was observed that the TC callback may be missed in the uart isr, basically when
// the jrpin5's uart isr priority is too low. This caused the jrpin5 loop to get stuck in STATE_TRANSMITING,
// and not even channel data would be received anymore (= very catastrophic). This code avoids this.
// With proper isr priorities, the issue is mainly gone, but the code remains, as safety net.

void tPin5BridgeBase::CheckAndRescue(void)
{
    uint32_t tnow_ms = millis32();

    if (state < STATE_TRANSMITING) {
        nottransmiting_tlast_ms = tnow_ms;
    } else {
        if (tnow_ms - nottransmiting_tlast_ms > 20) { // we are stuck, so rescue
#ifdef TX_FRM303_F072CB
            gpio_low(IO_PB9);
#endif
#if defined TX_DIY_SXDUAL_MODULE02_G491RE || defined TX_DIY_E28DUAL_MODULE02_G491RE || defined TX_DIY_E22DUAL_MODULE02_G491RE
            gpio_high(IO_PA0);
#endif
            state = STATE_IDLE;
            pin5_tx_enable(false);
#ifdef ESP32
            // ??
#else
            LL_USART_DisableIT_TC(UART_UARTx);
            LL_USART_ClearFlag_TC(UART_UARTx);
#endif    
        }
    }
}


#endif // JRPIN5_INTERFACE_H
