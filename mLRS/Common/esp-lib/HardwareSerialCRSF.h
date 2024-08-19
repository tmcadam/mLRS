#include <HardwareSerial.h>
#include "driver/uart.h"

#if !CONFIG_DISABLE_HAL_LOCKS
#define HSERIAL_MUTEX_LOCK() \
  do {                       \
  } while (xSemaphoreTake(_lock, portMAX_DELAY) != pdPASS)
#define HSERIAL_MUTEX_UNLOCK() xSemaphoreGive(_lock)
#else
#define HSERIAL_MUTEX_LOCK()
#define HSERIAL_MUTEX_UNLOCK()
#endif

#ifndef ARDUINO_SERIAL_SENDCB_TASK_STACK_SIZE
#define ARDUINO_SERIAL_SENDCB_TASK_STACK_SIZE 2048
#endif

#ifndef ARDUINO_SERIAL_SENDCB_TASK_PRIORITY
#define ARDUINO_SERIAL_SENDCB_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#endif

#ifndef ARDUINO_SERIAL_SENDCB_TASK_RUNNING_CORE
#define ARDUINO_SERIAL_SENDCB_TASK_RUNNING_CORE -1
#endif

typedef std::function<void(void)> OnSendCb;

class HardwareSerialCRSF : public HardwareSerial
{
public:
    HardwareSerialCRSF(uint8_t uart_nr) : HardwareSerial(uart_nr) {}
    void onSend(OnSendCb function);
    size_t write(uint8_t);
    size_t write(const uint8_t *buffer, size_t size);
protected:
    OnSendCb _onSendCB;
    static void _uartSendCBTask(void *args);
    void _createSendCBTask(void *args);
    void _destroySendCBTask(void); 
    TaskHandle_t _sendCBTask = NULL;


private:
    uint8_t _inSend = 0;
};


void HardwareSerialCRSF::_createSendCBTask(void *args) {
  // Creating UART event Task
  xTaskCreateUniversal(
    _uartSendCBTask, "uart_sendcb_task", ARDUINO_SERIAL_SENDCB_TASK_STACK_SIZE, this, ARDUINO_SERIAL_SENDCB_TASK_PRIORITY, &_sendCBTask,
    ARDUINO_SERIAL_SENDCB_TASK_RUNNING_CORE
  );

  if (_sendCBTask == NULL) {
    log_e(" -- UART%d Send Callback Task not Created!", _uart_nr);
  }
}

void HardwareSerialCRSF::_destroySendCBTask(void) {
  if (_sendCBTask != NULL) {
    vTaskDelete(_sendCBTask);
    _sendCBTask = NULL;
  }
}

void HardwareSerialCRSF::_uartSendCBTask(void *args) {
  HardwareSerialCRSF *uart = (HardwareSerialCRSF *)args;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  // maybe short delay needed here
  for(;;) {
    if (uart->_inSend) {
      // maybe short delay needed here
      uart->_inSend = 0;
      uart_wait_tx_done(uart->_uart_nr, portMAX_DELAY);
      uart->_onSendCB();
    }
    vTaskDelay(xDelay);
  }
  vTaskDelete(NULL);
}

void HardwareSerialCRSF::onSend(OnSendCb function) {

  HSERIAL_MUTEX_LOCK();
  // function may be NULL to cancel onSend() from its respective task
  _onSendCB = function;

  // setting the callback to NULL will just disable it
  if (_onSendCB != NULL) {
    // this method can be called after Serial.begin(), therefore it shall create the event task
    if (_uart != NULL && _sendCBTask == NULL) {
      _inSend = 0;
      _createSendCBTask(this);  // Create event task
    }
  } else {
    _destroySendCBTask();
  }
  HSERIAL_MUTEX_UNLOCK();
}

size_t HardwareSerialCRSF::write(uint8_t c) {
  uartWrite(_uart, c);
  if(_sendCBTask != NULL){
    _inSend = 1;
  }
  return 1;
}

size_t HardwareSerialCRSF::write(const uint8_t *buffer, size_t size) {
  uartWriteBuf(_uart, buffer, size);
  if(_sendCBTask != NULL){
    _inSend = 1;
  }
  return size;
}