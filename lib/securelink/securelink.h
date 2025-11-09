#ifndef SECURELINK_H
#define SECURELINK_H

#include "datalink.h"
#include "radio.h"

struct SecureLinkConfig
{
  GpioConfig gpio;
  RadioConfig radio;
  DataLinkConfig dll;
};

template <typename TStorage> class SecureLink
{
public:
  explicit SecureLink(const SecureLinkConfig &cfg, TStorage &storage)
      : _radio(cfg.gpio, cfg.radio), _dataLink(_radio, cfg.dll, storage)
  {
  }

  int begin(void) { return _dataLink.begin(); };

  int send(const Packet &pkt)
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

  int receive(Packet &pkt)
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

  int receive(Packet &pkt, uint32_t timeoutMs)
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

private:
  Radio _radio;
  DataLink<TStorage> _dataLink;
};

#endif // SECURELINK_H