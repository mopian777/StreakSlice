// Simulation-only causal lineage observer. Does not implement a scheduling policy.
#pragma once
#include "deadline_hooks.h"
#include "ns3/nr-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include <map>
#include <memory>
#include <fstream>
#include <array>
#include <algorithm>

namespace ns3 {
class V4LineageTag : public Tag {
public:
 uint64_t key=0;
 static TypeId GetTypeId() { static TypeId t=TypeId("ns3::V4LineageTag").SetParent<Tag>().AddConstructor<V4LineageTag>();return t; }
 TypeId GetInstanceTypeId() const override { return GetTypeId(); }
 uint32_t GetSerializedSize() const override { return 8; }
 void Serialize(TagBuffer b) const override { b.WriteU64(key); }
 void Deserialize(TagBuffer b) override { key=b.ReadU64(); }
 void Print(std::ostream& o) const override { o<<key; }
};
class V4Telemetry {
public:
 struct Entry { unsigned slice; uint32_t total,remaining,handed=0,dropped=0;int64_t generated,deadline;bool received=false; };
 struct Bearer { uint64_t bytes=0,count=0; };
 std::map<uint64_t,Entry> entries;
 std::map<std::pair<uint16_t,uint8_t>,Bearer> queues;
 std::ofstream events,snapshots;
 uint64_t checks=0,fragments=0;
 explicit V4Telemetry(const std::string& prefix) {
  if(prefix.empty()) return;
  events.open(prefix+"_v4_events.csv");snapshots.open(prefix+"_v4_state.csv");
  NS_ABORT_MSG_IF(!events.is_open()||!snapshots.is_open(),"V4 output failure");
  events<<"time_ns,event,key,slice,bytes,remaining,rnti,lcid,generated_ns,deadline_ns\n";
  snapshots<<"time_ns,slice,queued_bytes,queued_sdus,expired_bytes,slack_0_1ms,slack_1_2ms,slack_2_5ms,slack_5_10ms,slack_10_20ms,slack_20_50ms,slack_50_100ms,slack_100_500ms,slack_gt500ms,handed_unconfirmed_bytes\n";
 }
 void Log(const char* kind,uint64_t key,unsigned slice,uint64_t bytes,uint64_t remaining,uint16_t rnti,uint8_t lcid,int64_t gen=0,int64_t deadline=0) {
  if(events.is_open()) events<<Simulator::Now().GetNanoSeconds()<<','<<kind<<','<<key<<','<<slice<<','<<bytes<<','<<remaining<<','<<rnti<<','<<unsigned(lcid)<<','<<gen<<','<<deadline<<'\n';
 }
 uint64_t Ingress(Ptr<Packet> p,unsigned slice,uint16_t rnti,uint8_t lcid) {
  auto copy=p->Copy();NrPdcpHeader ph;Ipv4Header ip;UdpHeader udp;SeqTsHeader seq;
  copy->RemoveHeader(ph);copy->RemoveHeader(ip);copy->RemoveHeader(udp);copy->RemoveHeader(seq);
  NS_ABORT_MSG_IF(ip.GetProtocol()!=17||udp.GetDestinationPort()!=1234+slice||seq.GetTs()>Simulator::Now(),"V4 unsupported SDU identity");
  uint64_t key=(uint64_t(udp.GetDestinationPort())<<32)|seq.GetSeq();
  int64_t deadline=seq.GetTs().GetNanoSeconds()+std::array<int64_t,3>{10000000,100000000,500000000}[slice];
  DeadlineMetaTag dt;dt.deadline=deadline;p->AddPacketTag(dt);
  auto ok=entries.emplace(key,Entry{slice,p->GetSize(),p->GetSize(),0,0,seq.GetTs().GetNanoSeconds(),deadline,false}).second;
  NS_ABORT_MSG_IF(!ok,"V4 duplicate SDU");V4LineageTag tag;tag.key=key;p->AddByteTag(tag);
  auto& q=queues[{rnti,lcid}];q.bytes+=p->GetSize();q.count++;
  Log("in",key,slice,p->GetSize(),p->GetSize(),rnti,lcid,seq.GetTs().GetNanoSeconds(),deadline);return key;
 }
 void Transfer(uint64_t key,uint32_t bytes,bool drop,uint16_t rnti,uint8_t lcid) {
  auto it=entries.find(key);NS_ABORT_MSG_IF(it==entries.end(),"V4 unknown SDU");auto& e=it->second;
  NS_ABORT_MSG_IF(bytes==0||bytes>e.remaining||e.received,"V4 invalid SDU transfer");
  auto& q=queues.at({rnti,lcid});NS_ABORT_MSG_IF(q.bytes<bytes||!q.count,"V4 negative queue");
  e.remaining-=bytes;q.bytes-=bytes;if(!e.remaining)q.count--;
  if(drop)e.dropped+=bytes;else e.handed+=bytes;
  NS_ABORT_MSG_IF(e.total!=e.remaining+e.handed+e.dropped,"V4 conservation failure");
  Log(drop?(DeadlineDropInProgress()?"expire":"drop"):"out",key,e.slice,bytes,e.remaining,rnti,lcid);
 }
 void Depart(Ptr<const Packet> p,uint16_t rnti,uint8_t lcid) {
  NrRlcHeader h;p->PeekHeader(h);uint64_t bytes=0;auto it=p->GetByteTagIterator();
  while(it.HasNext()) { auto item=it.Next();if(item.GetTypeId()!=V4LineageTag::GetTypeId())continue;
   V4LineageTag tag;item.GetTag(tag);auto n=item.GetEnd()-item.GetStart();bytes+=n;Transfer(tag.key,n,false,rnti,lcid);fragments++;
  }
  NS_ABORT_MSG_IF(bytes+h.GetSerializedSize()!=p->GetSize(),"V4 uncovered or multiply tagged RLC bytes");
 }
 void Drop(Ptr<const Packet> p,uint16_t rnti,uint8_t lcid) {
  auto it=p->GetByteTagIterator();uint64_t bytes=0;
  while(it.HasNext()){auto item=it.Next();if(item.GetTypeId()!=V4LineageTag::GetTypeId())continue;V4LineageTag tag;item.GetTag(tag);auto n=item.GetEnd()-item.GetStart();bytes+=n;Transfer(tag.key,n,true,rnti,lcid);}
  NS_ABORT_MSG_IF(bytes!=p->GetSize(),"V4 malformed drop");
 }
 void Buffer(const NrMacSapProvider::ReportBufferStatusParameters& p) {
  auto& q=queues[{p.rnti,p.lcid}];NS_ABORT_MSG_IF(q.bytes+2*q.count!=p.txQueueSize,"V4 RLC queue mismatch");checks++;
  Log("bsr",0,0,p.txQueueSize,q.bytes,p.rnti,p.lcid);
 }
 void Rx(Ptr<const Packet> p,unsigned slice) {
  SeqTsHeader h;p->PeekHeader(h);uint64_t key=(uint64_t(1234+slice)<<32)|h.GetSeq();
  auto it=entries.find(key);NS_ABORT_MSG_IF(it==entries.end(),"V4 receive without ingress");auto& e=it->second;
  NS_ABORT_MSG_IF(e.received||e.remaining||e.dropped||e.handed!=e.total,"V4 invalid receive lineage");e.received=true;
  Log("rx",key,slice,e.total,0,0,0);
 }
 void RxU(Ptr<const Packet> p){Rx(p,0);}void RxE(Ptr<const Packet> p){Rx(p,1);}void RxM(Ptr<const Packet> p){Rx(p,2);}
 void Snapshot(Time t) {
  NS_ABORT_MSG_IF(t!=Simulator::Now(),"V4 noncausal snapshot");
  Log("snapshot",0,0,0,0,0,0);
  for(unsigned s=0;s<3;++s) {
   uint64_t q=0,n=0,expired=0,unconfirmed=0;std::array<uint64_t,9> hist{};
   const std::array<int64_t,8> edges{1000000,2000000,5000000,10000000,20000000,50000000,100000000,500000000};
   for(const auto& kv:entries){const auto& e=kv.second;if(e.slice!=s)continue;
    if(!e.received)unconfirmed+=e.handed;
    if(!e.remaining)continue;q+=e.remaining;n++;
    int64_t slack=e.deadline-t.GetNanoSeconds();
    if(slack<0)expired+=e.remaining;else hist[std::upper_bound(edges.begin(),edges.end(),slack)-edges.begin()]+=e.remaining;
   }
   snapshots<<t.GetNanoSeconds()<<','<<s<<','<<q<<','<<n<<','<<expired;
   for(auto h:hist)snapshots<<','<<h;snapshots<<','<<unconfirmed<<'\n';
  }
  snapshots.flush();events.flush();
 }
 struct Proxy : NrRlcSapProvider,NrMacSapProvider {
  V4Telemetry* owner;unsigned slice;uint16_t rnti;uint8_t lcid;NrRlcSapProvider* rlc;NrMacSapProvider* mac;
  Proxy(V4Telemetry* o,unsigned s,uint16_t r,uint8_t l,NrRlcSapProvider* rp,NrMacSapProvider* mp):owner(o),slice(s),rnti(r),lcid(l),rlc(rp),mac(mp){}
  void TransmitPdcpPdu(NrRlcSapProvider::TransmitPdcpPduParameters p) override {NS_ABORT_MSG_IF(p.rnti!=rnti||p.lcid!=lcid,"V4 bearer mismatch");owner->Ingress(p.pdcpPdu,slice,rnti,lcid);rlc->TransmitPdcpPdu(p);}
  void TransmitPdu(NrMacSapProvider::TransmitPduParameters p) override {owner->Depart(p.pdu,rnti,lcid);mac->TransmitPdu(p);}
  void ReportBufferStatus(NrMacSapProvider::ReportBufferStatusParameters p) override {owner->Buffer(p);mac->ReportBufferStatus(p);}
  void OnDrop(Ptr<const Packet> p){owner->Drop(p,rnti,lcid);}
 };
 std::vector<std::unique_ptr<Proxy>> proxies;
 void Attach(Ptr<NrGnbNetDevice> gnb,NetDeviceContainer u,NetDeviceContainer e,NetDeviceContainer m) {
  std::array<NetDeviceContainer,3> slices{u,e,m};
  for(unsigned s=0;s<3;++s){auto ue=DynamicCast<NrUeNetDevice>(slices[s].Get(0));uint16_t rnti=ue->GetRrc()->GetRnti();ObjectMapValue bearers;
   gnb->GetRrc()->GetUeManager(rnti)->GetAttribute("DataRadioBearerMap",bearers);
   for(auto it=bearers.Begin();it!=bearers.End();++it){auto info=it->second->GetObject<NrDataRadioBearerInfo>();auto rlc=DynamicCast<NrRlcUm>(info->m_rlc);
    NS_ABORT_MSG_IF(!rlc||!info->m_pdcp,"V4 requires UM plus PDCP");
    auto proxy=std::make_unique<Proxy>(this,s,rnti,info->m_logicalChannelIdentity,rlc->GetNrRlcSapProvider(),gnb->GetBwpManager()->GetNrMacSapProvider());
    BooleanValue discard;rlc->GetAttribute("EnablePdcpDiscarding",discard);NS_ABORT_MSG_IF(discard.Get(),"V4 drop audit requires no pseudo-discard");
    NS_ABORT_MSG_IF(!rlc->TraceConnectWithoutContext("TxDrop",MakeCallback(&Proxy::OnDrop,proxy.get())),"V4 cannot observe drops");
    info->m_pdcp->SetNrRlcSapProvider(proxy.get());rlc->SetNrMacSapProvider(proxy.get());proxies.push_back(std::move(proxy));
   }
  }
 }
};
class V4NullMac : public NrMacSapProvider {
public:
 uint32_t pdus=0;
 void TransmitPdu(TransmitPduParameters) override {pdus++;}
 void ReportBufferStatus(ReportBufferStatusParameters) override {}
};
inline int V4SelfTest() {
 V4Telemetry obs("");V4NullMac mac;auto rlc=CreateObject<NrRlcUm>();rlc->SetRnti(1);rlc->SetLcId(3);rlc->SetAttribute("MaxTxBufferSize",UintegerValue(500));rlc->SetAttribute("EnablePdcpDiscarding",BooleanValue(false));
 V4Telemetry::Proxy proxy(&obs,0,1,3,rlc->GetNrRlcSapProvider(),&mac);rlc->SetNrMacSapProvider(&proxy);
 rlc->TraceConnectWithoutContext("TxDrop",MakeCallback(&V4Telemetry::Proxy::OnDrop,&proxy));
 for(unsigned i=0;i<4;++i){auto p=Create<Packet>(188);SeqTsHeader h;h.SetSeq(i);p->AddHeader(h);UdpHeader u;u.SetDestinationPort(1234);p->AddHeader(u);Ipv4Header ip;ip.SetProtocol(17);ip.SetPayloadSize(p->GetSize());p->AddHeader(ip);NrPdcpHeader pdcp;pdcp.SetSequenceNumber(i);p->AddHeader(pdcp);proxy.TransmitPdcpPdu({p,1,3});}
 NS_ABORT_MSG_IF(obs.entries.size()!=4||obs.queues.at({1,3}).count!=2,"V4 admission test failed");
 rlc->DoNotifyTxOpportunity({80,0,0,0,1,3});rlc->DoNotifyTxOpportunity({1000,0,0,0,1,3});
 NS_ABORT_MSG_IF(obs.queues.at({1,3}).bytes!=0||obs.fragments<3||mac.pdus!=2||obs.checks!=4,"V4 segmentation/concatenation test failed");
 Simulator::Destroy();std::cout<<"V4_SELF_TEST_PASS: admission, overflow, segmentation, concatenation, byte conservation and buffer reports\n";return 0;
}
} // namespace ns3
