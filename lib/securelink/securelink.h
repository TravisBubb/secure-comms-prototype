#ifndef SECURELINK_H
#define SECURELINK_H

#include "radio.h"
#include "datalink.h"

struct SecureLinkConfig
{
  GpioConfig gpio;
  RadioConfig radio;
  DataLinkConfig dll;
};

class SecureLink
{
public:
  explicit SecureLink(const SecureLinkConfig &cfg);

  int begin(void);
  int send(const Packet &pkt);
  int receive(Packet &pkt);
  int receive(Packet &pkt, uint32_t timeoutMs);

private:
  Radio _radio;
  DataLink _dataLink;
};

#endif // SECURELINK_H