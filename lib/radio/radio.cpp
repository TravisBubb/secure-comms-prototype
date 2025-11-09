#include "radio.h"
#include "frame.h"
#include <string.h>

#ifndef UNIT_TEST
#include <Arduino.h>
#include <RadioLib.h>
#endif

#ifdef UNIT_TEST
SerialMock Serial;
#endif

Radio::Radio(const GpioConfig &gpioCfg, const RadioConfig &radioCfg)
    : _gpioCfg(gpioCfg), _radioCfg(radioCfg),
#ifdef UNIT_TEST
      _spi(), _module(), _radio(&_module)
#else
      _spi(HSPI), _module(gpioCfg.nssPin, gpioCfg.dio1Pin, gpioCfg.rstPin, gpioCfg.busyPin, _spi),
      _radio(&_module)
#endif
{
}

int Radio::begin(void)
{
  Serial.println("[Radio] Begin called");

  Serial.printf("[Radio] Initializing SPI with SCK=%d, MISO=%d, MOSI=%d, NSS=%d\n",
                _gpioCfg.sckPin,
                _gpioCfg.misoPin,
                _gpioCfg.mosiPin,
                _gpioCfg.nssPin);
  _spi.begin(_gpioCfg.sckPin, _gpioCfg.misoPin, _gpioCfg.mosiPin, _gpioCfg.nssPin);

  Serial.printf("[Radio] Config: freq=%d Hz, bw=%d Hz, sf=%d, cr=%d, sync=0x%02X, power=%d dBm, "
                "preamble=%d\n",
                _radioCfg.freqHz,
                _radioCfg.bwHz,
                _radioCfg.sf,
                _radioCfg.crDen,
                _radioCfg.syncWord,
                _radioCfg.powerDbm,
                _radioCfg.preambleLen);

  float freqMHz = _radioCfg.freqHz / 1e6;
  float bwKHz = _radioCfg.bwHz / 1e3;
  Serial.printf("[Radio] Converted: freq=%.3f MHz, bw=%.1f kHz\n", freqMHz, bwKHz);

  int state = _radio.begin(freqMHz,
                           bwKHz,
                           _radioCfg.sf,
                           _radioCfg.crDen,
                           _radioCfg.syncWord,
                           _radioCfg.powerDbm,
                           _radioCfg.preambleLen);

  Serial.printf("[Radio] _radio.begin() returned %d\n", state);

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println("[Radio] Initialization successful");
    return 0;
  }

  Serial.println("[Radio] Initialization failed");
  return -1;
}

int Radio::transmit(const uint8_t *data, size_t len)
{
  if (!data || len == 0) return -1;
  int state = _radio.transmit(data, len);
  if (state != 0) return state;

  Serial.println("Sent bytes:");
  for (int i = 0; i < len; ++i)
    Serial.printf("0x%02X ", data[i]);
  Serial.println();

  return 0;
}

int Radio::receive(uint8_t *out, size_t *len)
{
  if (!out || !len || *len == 0) return -1;

  int state = _radio.receive(out, *len);
  if (state != 0) return state;

  size_t rcvLen = _radio.getPacketLength();

  Serial.println("Received bytes:");
  for (int i = 0; i < rcvLen; ++i)
    Serial.printf("0x%02X ", out[i]);
  Serial.println();

  *len = rcvLen;

  return 0;
}

int Radio::receive(uint8_t *out, size_t *len, uint32_t timeoutMs)
{
  if (!out || !len || *len == 0) return -1;

  int state =  _radio.receive(out, *len, timeoutMs);
  if (state != 0) return state;

  size_t rcvLen = _radio.getPacketLength();

  Serial.println("Received bytes:");
  for (int i = 0; i < rcvLen; ++i)
    Serial.printf("0x%02X ", out[i]);
  Serial.println();

  *len = rcvLen;

  return 0;
}