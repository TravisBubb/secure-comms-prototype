#include "radio.h"
#include "frame.h"
#include <Arduino.h>
#include <RadioLib.h>
#include <string.h>

Radio::Radio(const GpioConfig &gpioCfg, const RadioConfig &radioCfg)
    : _gpioCfg(gpioCfg), _radioCfg(radioCfg), _spi(HSPI),
      _module(gpioCfg.nssPin, gpioCfg.dio1Pin, gpioCfg.rstPin, gpioCfg.busyPin, _spi),
      _radio(&_module)
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

int Radio::transmit(const Packet &pkt)
{
  Frame f;

  f.ver = 0x01;
  f.flags = 0x00; // TODO properly handle flags (see docs/packets.md for specification)
  f.dev_id = _devId;
  f.seq = _seq; // TODO generate unique seq

  // TODO: add fragmentation logic here if pkt.payload_len > MAX_PAYLOAD_LEN (200)
  if (pkt.payloadLen > MAX_FRAME_PAYLOAD_LEN) return -1;
  f.payloadLen = pkt.payloadLen;

  // TODO: encrypt the payload here

  memcpy(&f.payload, pkt.payload, pkt.payloadLen);

  // TODO: set real auth tag here
  memset(&f.auth_tag, 0xFF, 16);

  // TODO: calculate real CRC16
  f.crc = 0x1234;

  uint8_t buf[sizeof(Frame)];
  size_t len;
  if (!frame_serialize(f, buf, &len)) return -1;

  Serial.printf("Serialized length: %d\n", len);
  for (size_t i = 0; i < len; ++i)
    Serial.printf("%02X ", buf[i]);
  Serial.println();

  int state = _radio.transmit(buf, len);
  if (state == RADIOLIB_ERR_NONE) return 0;

  // TODO: handle different error codes
  return -1;
}

int Radio::receive(Packet &pkt)
{
  return 0;
}

int Radio::receive(Packet &pkt, uint32_t timeoutMs)
{
  Frame f;
  uint8_t buf[sizeof(Frame)];

  int state = _radio.receive(buf, sizeof(Frame), timeoutMs);
  if (state != RADIOLIB_ERR_NONE) return state; // TODO: custom error code

  size_t len = _radio.getPacketLength();
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;

  Serial.print("\n[info] Received packet: ");
  for (size_t i = 0; i < len; ++i)
    Serial.printf("%02X ", buf[i]);
  Serial.printf(", length=%d\n", len);

  if (!frame_deserialize(f, buf, len)) return -1;

  // TODO: validate payload len to prevent overflows

  // TODO: handle fragmentation & reassembly

  // TODO: decrypt payload

  // TODO: validate CRC

  pkt.payloadLen = f.payloadLen;
  memcpy(&pkt.payload, f.payload, f.payloadLen);

  // TODO: check flags to see if an ACK should be sent back

  return 0;
}