#ifndef IN_MEMORY_STORAGE_H
#define IN_MEMORY_STORAGE_H

#include <stdint.h>

// Implements interface defined in storage_traits.h
class InMemoryStorage
{
public:
  InMemoryStorage(uint16_t devId) : _devId(devId) {}

  uint16_t loadDeviceId() const { return _devId; }
  void saveDeviceId(uint16_t devId) { _devId = devId; }

  uint32_t loadSequenceNumber() const { return _seq; }
  void saveSequenceNumber(uint32_t seq) { _seq = seq; }

private:
  uint16_t _devId = 0;
  uint32_t _seq = 0;
};

#endif // IN_MEMORY_STORAGE_H