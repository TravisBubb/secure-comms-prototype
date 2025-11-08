#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stddef.h>

#ifdef UNIT_TEST
#define RADIOLIB_ERR_NONE 0
#else
#include <RadioLib.h>
#endif

#define MAX_PACKET_PAYLOAD 1024

struct GpioConfig
{
  int misoPin;
  int mosiPin;
  int nssPin;
  int sckPin;
  int dio1Pin;
  int rstPin;
  int busyPin;
};

struct RadioConfig
{
  int freqHz;
  int bwHz;
  int sf;
  int crDen;
  uint8_t syncWord;
  int powerDbm;
  int preambleLen;

  static constexpr RadioConfig defaultConfig() { return {915000000, 125000, 7, 5, 0x12, 10, 8}; }
};

struct Packet 
{
  uint16_t destId;
  uint8_t payload[MAX_PACKET_PAYLOAD];
  uint8_t payloadLen;
};

#ifdef UNIT_TEST
// Mock classes
class SPIClass {
public:
  void begin(int, int, int, int){}
};

class Module{};

class SX1262 {
public:
  SX1262(Module*){}
  int begin(float, float, int, int, uint8_t, int, int) { return 0; }
  int transmit(uint8_t *, size_t) { return 0; }
  int receive(uint8_t *, size_t, uint32_t) { return 0; }
  size_t getPacketLength() { return 0; }
};

class SerialMock {
public:
  void printf(const char* fmt, ...){}
  void print(const char *){}
  void println(const char *){}
  void println(){}
};

extern SerialMock Serial;
#endif

class Radio
{
public:
  Radio(const GpioConfig &spi, const RadioConfig &radio);

  int begin(void);
  int transmit(const Packet &pkt);
  int receive(Packet &pkt);
  int receive(Packet &pkt, uint32_t timeoutMs);

private:
  RadioConfig _radioCfg;
  GpioConfig _gpioCfg;

#ifdef UNIT_TEST
  SPIClass _spi;
  Module _module;
  SX1262 _radio;
#else
  SPIClass _spi;
  Module _module;
  SX1262 _radio;
#endif

  uint32_t _seq;
  uint16_t _devId;
};

#endif // RADIO_H