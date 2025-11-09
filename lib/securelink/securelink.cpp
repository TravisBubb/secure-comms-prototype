#include "securelink.h"
#include <string.h>

SecureLink::SecureLink(const SecureLinkConfig &cfg)
    : _radio(cfg.gpio, cfg.radio), _dataLink(_radio, cfg.dll)
{
}

int SecureLink::begin(void)
{
  return _dataLink.begin();
}

int SecureLink::send(const Packet &pkt)
{
  Frame f;

  if (pkt.payloadLen > MAX_FRAME_PAYLOAD_LEN) return -1;
  f.payloadLen = pkt.payloadLen;

  // TODO: encrypt payload

  memcpy(&f.payload, pkt.payload, pkt.payloadLen);

  // TODO: set real auth tag
  memset(&f.authTag, 0xFF, 16);

  return _dataLink.send(f);
}

int SecureLink::receive(Packet &pkt)
{
  Frame f;

  int state = _dataLink.receive(f);
  if (state != 0) return state;

  // TODO: validate auth tag

  // TODO: decrypt payload

  pkt.payloadLen = f.payloadLen;
  memcpy(&pkt.payload, f.payload, f.payloadLen);

  return 0;
}

int SecureLink::receive(Packet &pkt, uint32_t timeoutMs)
{
  Frame f;

  int state = _dataLink.receive(f, timeoutMs);
  if (state != 0) return state;

  // TODO: validate auth tag

  // TODO: decrypt payload

  pkt.payloadLen = f.payloadLen;
  memcpy(&pkt.payload, f.payload, f.payloadLen);

  return 0;
}