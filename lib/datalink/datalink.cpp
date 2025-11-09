#include "datalink.h"

DataLink::DataLink(Radio &radio, const DataLinkConfig &cfg) : _radio(radio), _cfg(cfg) { }

int DataLink::begin(void)
{
  return _radio.begin();
}

int DataLink::send(Frame &frame)
{
  frame.ver = 0x01;

  // TODO: properly handle flags based on config (see docs/packets.md for specification)
  frame.flags = 0x00; 

  // TODO: properly generate these values
  frame.dev_id = 0x1234;
  frame.seq = 0x11223344;

  // TODO: handle fragmentation

    uint8_t buf[sizeof(Frame)];
    size_t len;
    if (!frame_serialize(frame, buf, &len)) return -1;

    return _radio.transmit(buf, len);
}

int DataLink::receive(Frame &frame)
{
  return 0;
}

int DataLink::receive(Frame &frame, uint32_t timeoutMs)
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