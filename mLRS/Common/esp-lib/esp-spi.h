//*******************************************************
// Copyright (c) MLRS project
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP SPI Interface
//********************************************************
#ifndef ESPLIB_SPI_H
#define ESPLIB_SPI_H


#include <SPI.h>

SPIClass* spi = NULL;

//-- select functions

#ifndef SPI_SELECT_PRE_DELAY
  #define SPI_SELECT_PRE_DELAY
#endif
#ifndef SPI_SELECT_POST_DELAY
  #define SPI_SELECT_POST_DELAY
#endif
#ifndef SPI_DESELECT_PRE_DELAY
  #define SPI_DESELECT_PRE_DELAY
#endif
#ifndef SPI_DESELECT_POST_DELAY
  #define SPI_DESELECT_POST_DELAY
#endif


#ifdef SPI_CS_IO

static inline IRAM_ATTR void spi_select(void)
{
#if defined(ESP32) 
  GPIO.out_w1tc = ((uint32_t)1 << SPI_CS_IO);
#elif defined(ESP8266)
  GPOC - (1 << SPI_CS_IO);
#endif
  spi->beginTransaction(SPISettings(SPI_FREQUENCY, MSBFIRST, SPI_MODE0));
}


static inline IRAM_ATTR void spi_deselect(void)
{
  spi->endTransaction();
#if defined(ESP32) 
  GPIO.out_w1ts = ((uint32_t)1 << SPI_CS_IO);
#elif defined(ESP8266)
  GPOS - (1 << SPI_CS_IO);
#endif
}

IRAM_ATTR void spi_init(void)
{
  pinMode(SPI_CS_IO, OUTPUT);
  digitalWrite(SPI_CS_IO, HIGH);

  spi = new SPIClass();

#if defined(ESP32) 
  spi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, SPI_CS_IO);
#elif defined(ESP8266)
  spi->begin(); 
#endif

}

//-- transmit, transfer, read, write functions

// is blocking
IRAM_ATTR uint8_t spi_transmitchar(uint8_t c)
{
  return spi->transfer(c);
}

// is blocking
void spi_transfer(uint8_t* dataout, uint8_t* datain, uint16_t len)
{
  while (len) {
    *datain = spi_transmitchar(*dataout);
    dataout++;
    datain++;
    len--;
  }
}

// To utilize ESP SPI Buffer
IRAM_ATTR void spi_transferbytes(uint8_t* dataout, uint8_t* datain, uint8_t len)
{
  spi->transferBytes(dataout, datain, len);
}

// sends a dummy char, returns the received byte
// is blocking
static inline uint8_t spi_readchar(void)
{
  return spi_transmitchar(0xFF);
}


// sends a char, ignores the received byte
// is blocking
static inline void spi_writechar(uint8_t c)
{
  spi_transmitchar(c);
}

// sends a word, returns the received word
// is blocking
static inline uint16_t spi_transmitword(uint16_t w)
{
  return (((uint16_t)spi_transmitchar((uint8_t)(w >> 8))) << 8) + spi_transmitchar((uint8_t)(w));
}


// sends a dummy word, returns the received word
// is blocking
static inline uint16_t spi_readword(void)
{
  return (((uint16_t)spi_transmitchar(0xFF)) << 8) + spi_transmitchar(0xFF);
}


// sends a word, ignores the received word
// is blocking
static inline void spi_writeword(uint16_t w)
{
  spi_transmitchar((uint8_t)(w >> 8));
  spi_transmitchar((uint8_t)(w));
}


// is blocking
void spi_read(uint8_t* data, uint16_t len)
{
  while (len) {
    *data = spi_readchar();
    data++;
    len--;
  }
}


// is blocking
void spi_write(uint8_t* data, uint16_t len)
{
  while (len) {
    spi_writechar(*data);
    data++;
    len--;
  }
}


// is blocking
void spi_writecandread(uint8_t c, uint8_t* data, uint16_t datalen)
{
  spi_writechar(c);
  while (datalen) {
    *data = spi_readchar();
    data++;
    datalen--;
  }
}

#endif

#endif // ESPLIB_SPI_H