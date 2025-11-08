#ifndef RADIO_H
#define RADIO_H

#include <RadioLib.h>
#include <stdint.h>

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
  SPIClass _spi;
  Module _module;
  SX1262 _radio;

  uint32_t _seq;
  uint16_t _devId;
};

#endif // RADIO_H