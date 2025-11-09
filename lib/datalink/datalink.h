#ifndef DATALINK_H
#define DATALINK_H

#include "frame.h"
#include "radio.h"
#include "storage_traits.h"

struct DataLinkConfig
{
  bool enableFragmentation;
};

template <typename TStorage> class DataLink
{
  static_assert(is_storage<TStorage>::value,
                "TStorage must implement loadDeviceId(), saveDeviceId(), loadSequenceNumber(), and "
                "saveSequenceNumber() with correct types.");

public:
  explicit DataLink(Radio &radio, const DataLinkConfig &cfg, TStorage &storage)
      : _radio(radio), _cfg(cfg), _storage(storage)
  {
    _devId = _storage.loadDeviceId();
    _seq = _storage.loadSequenceNumber();
  }

  uint16_t getDeviceId(void) const { return _devId; };
  uint32_t getSequenceNumber(void) const { return _seq; };

  int begin(void) { return _radio.begin(); };

  int send(Frame &frame)
  {

    frame.ver = 0x01;

    // TODO: properly handle flags based on config (see docs/packets.md for specification)
    frame.flags = 0x00;

    // TODO: properly generate these values
    frame.dev_id = _devId;
    frame.seq = _seq;

    // TODO: handle fragmentation

    uint8_t buf[sizeof(Frame)];
    size_t len;
    if (!frame_serialize(frame, buf, &len)) return -1;

    // TODO: increment seq

    return _radio.transmit(buf, len);
  }

  int receive(Frame &frame) { return 0; }

  int receive(Frame &frame, uint32_t timeoutMs)
  {
    uint8_t buf[sizeof(Frame)];
    size_t len = sizeof(Frame);

    int state = _radio.receive(buf, &len, timeoutMs);
    if (state != 0) return state;

    if (!frame_deserialize(frame, buf, len)) return -1;

    // TODO: handle fragmentation & reassembly

    // TODO: check flags to see if an ACK should be sent back

    return 0;
  }

private:
  Radio &_radio;
  DataLinkConfig _cfg;
  TStorage &_storage;
  uint16_t _devId;
  uint32_t _seq;
};

#endif // DATALINK_H