//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP RxClock
//********************************************************
#ifndef ESP_RXCLOCK_H
#define ESP_RXCLOCK_H
#pragma once


#define USING_TIM_DIV1 true

#if defined(ESP8266)
  #include "ESP8266TimerInterrupt.h"
  ESP8266Timer ITimer;
#elif defined(ESP32) 
  hw_timer_t* timer0_cfg = nullptr;
#endif
// we use a 10us time base, so that overrun is 655 ms
// 65 ms would only be 3 packets
// I have tested it with 1us time base, and it also works fine, but hey
// Note that TIM2 may be 16 bit or 32 bit depending on which STM32 controller is used


#define CLOCK_SHIFT_10US          100 // 75 // 100 // 1 ms

volatile bool doPostReceive;

uint16_t CLOCK_PERIOD_10US; // does not change while isr is enabled, so no need for volatile

volatile uint32_t CNT_10us = 0;
volatile uint32_t CCR1 = 0;
volatile uint32_t CCR3 = 0;
volatile uint32_t MS_C = 0;

//-------------------------------------------------------
// Clock ISR
//-------------------------------------------------------

IRQHANDLER(
IRAM_ATTR void CLOCK_IRQHandler(void)
{
    CNT_10us++;

    // Call HAL_IncTick every 1ms
    if (CNT_10us == MS_C) {
        MS_C = CNT_10us + 100; 
        HAL_IncTick();
    }

    // this is at about when RX was or was supposed to be received
    if (CNT_10us == CCR1) { 
        CCR3 = CNT_10us + CLOCK_SHIFT_10US; // next doPostReceive
        CCR1 = CNT_10us + CLOCK_PERIOD_10US; // next tick
    }

    // this is 1 ms after RX was or was supposed to be received
    if (CNT_10us == CCR3) { 
        doPostReceive = true;
    }
})


//-------------------------------------------------------
// RxClock Class
//-------------------------------------------------------

class RxClockBase
{
  public:
    RxClockBase() {
        initialized = false;    
    }

    void Init(uint16_t period_ms);
    void SetPeriod(uint16_t period_ms);
    void Reset(void);

  private:
    bool initialized = false;
};


void RxClockBase::Init(uint16_t period_ms)
{
    if (initialized) { 
        return; 
    }

    CLOCK_PERIOD_10US = period_ms * 100; // frame rate in units of 10us
    doPostReceive = false;
    timer0_cfg = timerBegin(0, 800, 1);  // Timer 0, APB clock is 80 Mhz | divide by 80 is 100 KHz / 10 us, count up
    timerAttachInterrupt(timer0_cfg, &CLOCK_IRQHandler, true);
    timerAlarmWrite(timer0_cfg, 1, true);
    timerAlarmEnable(timer0_cfg);
    Reset();

    initialized = true;
}

void RxClockBase::SetPeriod(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100;
}


void RxClockBase::Reset(void)
{
    if (!CLOCK_PERIOD_10US) while (1) {}

    __disable_irq(); // WHAT TO DFO HERE !!!!
    CCR1 = CNT_10us + CLOCK_PERIOD_10US;
    CCR3 = CNT_10us + CLOCK_SHIFT_10US;
    MS_C = CNT_10us + 100;
    __enable_irq(); // WHAT TO DFO HERE !!!!
}

#endif // ESP_RXCLOCK_H
