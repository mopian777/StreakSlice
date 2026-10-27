// Isolated, opt-in RLC extension. Does not change NrRlcUm object layout.
#pragma once
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include <cstdint>
#include <limits>
namespace ns3 {
class DeadlineMetaTag : public Tag {
 public:
  int64_t deadline = 0;
  static TypeId GetTypeId();
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 8; }
  void Serialize(TagBuffer b) const override { b.WriteU64(deadline); }
  void Deserialize(TagBuffer b) override { deadline = b.ReadU64(); }
  void Print(std::ostream& o) const override { o << deadline; }
};
void SetDeadlineExpiry(bool enabled);
bool DeadlineExpiryEnabled();
bool DeadlineDropInProgress();
void SetDeadlineDropInProgress(bool active);
uint64_t DeadlineExpiredPackets();
void CountDeadlineExpiredPacket();
// No future outcome: generated timestamp + fixed deadline is attached at ingress.
int64_t PacketDeadline(Ptr<const Packet> packet);
}
