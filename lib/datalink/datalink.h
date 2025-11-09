#ifndef DATALINK_H
#define DATALINK_H

#include "frame.h"
#include "radio.h"

struct DataLinkConfig
{
  bool enableCrc;
  bool enableFragmentation;
};

class DataLink
{
public:
  explicit DataLink(Radio &radio, const DataLinkConfig &cfg = {});

  int begin(void);
  int send(const Frame &frame);
  int receive(Frame &frame);
  int receive(Frame &frame, uint32_t timeoutMs);

private:
  Radio &_radio;
  DataLinkConfig _cfg;
};

#endif // DATALINK_H