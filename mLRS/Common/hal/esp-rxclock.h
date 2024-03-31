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

#if defined(ESP32)
  hw_timer_t* timer0_cfg = nullptr;
#endif

#define CLOCK_SHIFT_10US          100 // 75 // 100 // 1 ms
#define CLOCK_CNT_1MS             100 // 10us interval 10us x 100 = 1000us        

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
void IRAM_ATTR CLOCK_IRQHandler(void)
{
    CNT_10us++;

    // Call HAL_IncTick every 1ms
    if (CNT_10us == MS_C) {
        MS_C = CNT_10us + CLOCK_CNT_1MS; 
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
    void Init(uint16_t period_ms);
    void SetPeriod(uint16_t period_ms);
    void Reset(void);
    void disable_isr(void);

  private:
    bool initialized = false;
};


void RxClockBase::Init(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100; // frame rate in units of 10us
    doPostReceive = false;

    if (initialized) { 
        return; 
    }

    // Initialise the timer
#if defined(ESP32)
    timer0_cfg = timerBegin(0, 800, 1);  // Timer 0, APB clock is 80 Mhz | divide by 800 is 100 KHz / 10 us, count up    
    timerAttachInterrupt(timer0_cfg, &CLOCK_IRQHandler, true);
    timerAlarmWrite(timer0_cfg, 1, true);
    timerAlarmEnable(timer0_cfg);    
#elif defined(ESP8266)
    timer1_attachInterrupt(CLOCK_IRQHandler); 
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
    timer1_write(50); //5MHz (5 ticks/us - 1677721.4 us max), 50 ticks = 10us
#endif
    Reset();
    initialized = true;
}

void RxClockBase::disable_isr(void)
{
#if defined(ESP32)
    timerDetachInterrupt(timer0_cfg);
#elif defined(ESP8266)
    timer1_detachInterrupt(); 
#endif
    
}

void RxClockBase::SetPeriod(uint16_t period_ms)
{
    CLOCK_PERIOD_10US = period_ms * 100;
}

void RxClockBase::Reset(void)
{
    if (!CLOCK_PERIOD_10US) while (1) {}
    noInterrupts();
    CCR1 = CNT_10us + CLOCK_PERIOD_10US;
    CCR3 = CNT_10us + CLOCK_SHIFT_10US;
    MS_C = CNT_10us + CLOCK_CNT_1MS;
    interrupts();
}

#endif // ESP_RXCLOCK_H
