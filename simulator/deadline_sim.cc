// Copyright (c) 2022 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \ingroup examples
 * \file burstshield_nr_stage7.cc
 * \brief Frozen BurstShield-PD with hard per-slice DL symbol quotas.
 *
 * Forked from CTTC's cttc-nr-simple-qos-sched example (5G-LENA v3.3.y).
 * One gNB serves three UEs on a shared BWP, representing URLLC, eMBB and
 * mMTC service classes. The URLLC proxy experiences a configurable traffic
 * burst and all three flows experience a temporary gNB-power reduction.
 * FlowMonitor KPIs are sampled into fixed windows; UDP receive traces provide
 * per-packet delays for exploratory tail-risk analysis.
 * The traffic, impairment, and SLA settings are calibration proxies, not the
 * paper's final traffic models. The optional per-slice RLC caps are applied
 * identically to the frozen-PD and reactive control modes.
 *
 * \code{.unparsed}
$ ./ns3 run "burstshield_nr_stage7 --PrintHelp"
    \endcode
 *
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "v4_telemetry.h"
#include "cohort_compliance.h"
#include "umi_geometry.h"

namespace ns3
{
bool gFastDeadline = false;

// This TDMA scheduler enforces causal, per-epoch DL symbol ceilings; the
// unused action mass stays idle rather than being normalized away.
class BurstShieldShareScheduler : public NrMacSchedulerTdmaQos
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::BurstShieldShareScheduler")
                                .SetParent<NrMacSchedulerTdmaQos>()
                                .AddConstructor<BurstShieldShareScheduler>();
        return tid;
    }

    void ConfigureSlices(const std::array<uint16_t, 3>& rntis)
    {
        NS_ABORT_MSG_IF(rntis[0] == 0 || rntis[1] == 0 || rntis[2] == 0 ||
                            rntis[0] == rntis[1] || rntis[0] == rntis[2] ||
                            rntis[1] == rntis[2],
                        "Three distinct nonzero UE RNTIs are required");
        m_rntis = rntis;
        m_configured = true;
    }

    void SetShares(const std::array<double, 3>& shares)
    {
        const double total = shares[0] + shares[1] + shares[2];
        NS_ABORT_MSG_IF(!m_configured || !std::isfinite(total) || total > 1.0 + 1e-9 ||
                            std::any_of(shares.begin(), shares.end(), [](double value) {
                                return !std::isfinite(value) || value < 0;
                            }),
                        "Three finite nonnegative shares summing to at most one are required");
        m_shares = shares;
        m_epochSymbols.fill(0);
        m_epochScheduledBits.fill(0);
        m_epochAvailableSymbols = 0;
        m_epochPotentialBits = 0;
    }

    void SetCapacityScale(double scale)
    {
        NS_ABORT_MSG_IF(!std::isfinite(scale) || scale <= 0.0 || scale > 1.0,
                        "Capacity scale must lie in (0,1]");
        m_capacityScale = scale;
    }

    std::array<uint64_t, 3> GetEpochSymbols() const
    {
        return m_epochSymbols;
    }

    uint64_t GetEpochAvailableSymbols() const
    {
        return static_cast<uint64_t>(std::floor(m_epochAvailableSymbols * m_capacityScale));
    }

    uint64_t GetEpochPotentialBits() const
    {
        return static_cast<uint64_t>(std::floor(m_epochPotentialBits * m_capacityScale));
    }

    std::array<double, 3> GetQueueBits() const
    {
        std::array<double, 3> result{};
        for (const auto& item : m_bufferReports)
        {
            result[item.first.first] += 8.0 * item.second.first;
        }
        return result;
    }

    std::array<double, 3> GetOldestAgeMs() const
    {
        std::array<double, 3> result{};
        for (const auto& item : m_bufferReports)
        {
            if (item.second.first > 0)
            {
                result[item.first.first] = std::max(result[item.first.first], item.second.second);
            }
        }
        return result;
    }

    std::array<uint64_t, 3> GetEpochScheduledBits() const
    {
        return m_epochScheduledBits;
    }

    std::array<double, 3> GetLastMcs() const
    {
        return m_lastMcs;
    }

    double GetMaxMcsSeen() const
    {
        return m_maxMcsSeen;
    }

  protected:
    BeamSymbolMap AssignDLRBG(uint32_t symAvail, const ActiveUeMap& activeDl) const override
    {
        if (!m_configured)
        {
            return NrMacSchedulerTdmaQos::AssignDLRBG(symAvail, activeDl);
        }
        const auto mask = GetDlNotchedRbgMask();
        const uint32_t assignableRbgs = GetBandwidthInRbg() -
                                        std::count(mask.begin(), mask.end(), 0);
        NS_ABORT_MSG_IF(assignableRbgs == 0, "No assignable DL RBGs");
        m_epochAvailableSymbols += symAvail;
        BeamSymbolMap allocated;
        std::vector<std::pair<BeamId, UePtrAndBufferReq>> contenders;
        uint32_t bestFullWindowTbs = 0;
        for (const auto& beam : activeDl)
        {
            allocated[beam.first] = 0;
            for (const auto& ue : beam.second)
            {
                NS_ABORT_MSG_IF(SliceIndex(ue.first->m_rnti) < 0,
                                "Hard-quota scheduler saw an unregistered active UE");
                contenders.emplace_back(beam.first, ue);
                BeforeDlSched(ue, FTResources(assignableRbgs, 1));
                bestFullWindowTbs = std::max(
                    bestFullWindowTbs,
                    m_dlAmc->CalculateTbSize(ue.first->m_dlMcs,
                                             ue.first->m_dlRank,
                                             assignableRbgs * symAvail * GetNumRbPerRbg()));
            }
        }
        // Counterfactual full-slot PHY opportunity, before this policy's
        // symbol ceilings and independent of application-layer delivery.
        m_epochPotentialBits += 8ULL * bestFullWindowTbs;
        FTResources assigned(0, 0);
        for (uint32_t symbol = 0; symbol < symAvail; ++symbol)
        {
            int best = -1;
            long double bestDeficit = -1.0;
            double bestSlack = std::numeric_limits<double>::infinity();
            for (size_t position = 0; position < contenders.size(); ++position)
            {
                const auto& ue = contenders[position].second;
                const int index = SliceIndex(ue.first->m_rnti);
                const auto ceiling = static_cast<uint64_t>(
                    std::floor(m_epochAvailableSymbols * m_capacityScale *
                               m_shares[index] + 1e-9));
                if (m_epochSymbols[index] >= ceiling ||
                    NrMacSchedulerUeInfo::GetDlTBS(ue.first) >= std::max(ue.second, 10U))
                {
                    continue;
                }
                const long double deficit = static_cast<long double>(ceiling) -
                                            m_epochSymbols[index];
                const auto ages = GetOldestAgeMs();
                double slack = std::array<double,3>{10.,100.,500.}[index] - ages[index];
                if ((!gFastDeadline && (deficit > bestDeficit ||
                    (deficit == bestDeficit && best >= 0 && ue.first->m_rnti < contenders[best].second.first->m_rnti))) ||
                    (gFastDeadline && (slack < bestSlack || (slack == bestSlack && deficit > bestDeficit))))
                {
                    best = static_cast<int>(position);
                    bestDeficit = deficit;
                    bestSlack = slack;
                }
            }
            if (best < 0)
            {
                break; // The action's idle share or an unsatisfied demand cap remains idle.
            }
            auto& chosen = contenders[best];
            NrMacSchedulerUeInfo::GetDlRBG(chosen.second.first) += assignableRbgs;
            NrMacSchedulerUeInfo::GetDlSym(chosen.second.first) += 1;
            assigned.m_rbg += assignableRbgs;
            assigned.m_sym += 1;
            AssignedDlResources(chosen.second, FTResources(assignableRbgs, 1), assigned);
            ++allocated[chosen.first];
            for (size_t position = 0; position < contenders.size(); ++position)
            {
                if (static_cast<int>(position) != best)
                {
                    NotAssignedDlResources(contenders[position].second,
                                           FTResources(assignableRbgs, 1),
                                           assigned);
                }
            }
        }
        return allocated;
    }

    void DoSchedDlRlcBufferReq(
        const NrMacSchedSapProvider::SchedDlRlcBufferReqParameters& params) override
    {
        NrMacSchedulerTdmaQos::DoSchedDlRlcBufferReq(params);
        const int index = SliceIndex(params.m_rnti);
        if (index >= 0 && params.m_logicalChannelIdentity >= 3)
        {
            m_bufferReports[{index, params.m_logicalChannelIdentity}] = {
                static_cast<double>(params.m_rlcTransmissionQueueSize),
                static_cast<double>(params.m_rlcTransmissionQueueHolDelay)};
        }
    }

    std::function<bool(const UePtrAndBufferReq&, const UePtrAndBufferReq&)>
    GetUeCompareDlFn() const override
    {
        return [this](const UePtrAndBufferReq& left, const UePtrAndBufferReq& right) {
            const int li = SliceIndex(left.first->m_rnti);
            const int ri = SliceIndex(right.first->m_rnti);
            if (li < 0 || ri < 0)
            {
                return NrMacSchedulerUeInfoQos::CompareUeWeightsDl(left, right);
            }
            if (gFastDeadline)
            {
                const auto ages = GetOldestAgeMs();
                const std::array<double,3> deadlines{10.,100.,500.};
                const double ls = deadlines[li]-ages[li], rs = deadlines[ri]-ages[ri];
                if (ls != rs) return ls < rs;
            }
            const long double lhs = static_cast<long double>(m_epochSymbols[li]) /
                                    std::max(m_shares[li], 1e-12);
            const long double rhs = static_cast<long double>(m_epochSymbols[ri]) /
                                    std::max(m_shares[ri], 1e-12);
            if (lhs != rhs)
            {
                return lhs < rhs;
            }
            return left.first->m_rnti < right.first->m_rnti;
        };
    }

    void AssignedDlResources(const UePtrAndBufferReq& ue,
                             const FTResources& assigned,
                             const FTResources& totAssigned) const override
    {
        NrMacSchedulerTdmaQos::AssignedDlResources(ue, assigned, totAssigned);
        const int index = SliceIndex(ue.first->m_rnti);
        if (index >= 0)
        {
            m_epochSymbols[index] += assigned.m_sym;
            m_epochScheduledBits[index] += 8ULL * ue.first->m_dlTbSize;
            m_lastMcs[index] = ue.first->m_dlMcs;
            m_maxMcsSeen = std::max(m_maxMcsSeen, m_lastMcs[index]);
        }
    }

  private:
    int SliceIndex(uint16_t rnti) const
    {
        if (!m_configured)
        {
            return -1;
        }
        for (int i = 0; i < 3; ++i)
        {
            if (m_rntis[i] == rnti)
            {
                return i;
            }
        }
        return -1;
    }

    bool m_configured{false};
    std::array<uint16_t, 3> m_rntis{};
    std::array<double, 3> m_shares{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    mutable std::array<uint64_t, 3> m_epochSymbols{};
    mutable uint64_t m_epochAvailableSymbols{0};
    mutable uint64_t m_epochPotentialBits{0};
    mutable std::array<uint64_t, 3> m_epochScheduledBits{};
    mutable std::array<double, 3> m_lastMcs{};
    mutable double m_maxMcsSeen{0};
    double m_capacityScale{1.0};
    std::map<std::pair<int, uint8_t>, std::pair<double, double>> m_bufferReports;
};

NS_OBJECT_ENSURE_REGISTERED(BurstShieldShareScheduler);

} // namespace ns3

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("BurstShieldNrStage7");

namespace
{

void
InitializeShareScheduler(Ptr<BurstShieldShareScheduler> scheduler,
                         const NetDeviceContainer& urllcDevices,
                         const NetDeviceContainer& embbDevices,
                         const NetDeviceContainer& mmtcDevices)
{
    const std::array<uint16_t, 3> rntis = {
        DynamicCast<NrUeNetDevice>(urllcDevices.Get(0))->GetRrc()->GetRnti(),
        DynamicCast<NrUeNetDevice>(embbDevices.Get(0))->GetRrc()->GetRnti(),
        DynamicCast<NrUeNetDevice>(mmtcDevices.Get(0))->GetRrc()->GetRnti()};
    scheduler->ConfigureSlices(rntis);
    scheduler->SetShares({1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0});
}

void
ConfigurePerSliceRlc(Ptr<NrGnbNetDevice> gnb,
                     const NetDeviceContainer& urllcDevices,
                     const NetDeviceContainer& embbDevices,
                     const NetDeviceContainer& mmtcDevices,
                     const std::array<uint32_t, 3>& caps)
{
    const std::array<Ptr<NrUeNetDevice>, 3> ues = {
        DynamicCast<NrUeNetDevice>(urllcDevices.Get(0)),
        DynamicCast<NrUeNetDevice>(embbDevices.Get(0)),
        DynamicCast<NrUeNetDevice>(mmtcDevices.Get(0))};
    for (size_t slice = 0; slice < ues.size(); ++slice)
    {
        const uint16_t rnti = ues[slice]->GetRrc()->GetRnti();
        NS_ABORT_MSG_IF(rnti == 0, "Per-slice RLC cap requires a connected UE");
        Ptr<NrUeManager> manager = gnb->GetRrc()->GetUeManager(rnti);
        NS_ABORT_MSG_IF(!manager, "Missing gNB UE manager for per-slice RLC cap");
        ObjectMapValue bearers;
        manager->GetAttribute("DataRadioBearerMap", bearers);
        NS_ABORT_MSG_IF(bearers.GetN() < 2, "Expected default and dedicated DRBs per UE");
        uint32_t configured = 0;
        for (auto it = bearers.Begin(); it != bearers.End(); ++it)
        {
            Ptr<NrDataRadioBearerInfo> info = it->second->GetObject<NrDataRadioBearerInfo>();
            Ptr<NrRlcUm> rlc = DynamicCast<NrRlcUm>(info->m_rlc);
            NS_ABORT_MSG_IF(!rlc, "Per-slice diagnostic requires NR RLC UM bearers");
            rlc->SetAttribute("MaxTxBufferSize", UintegerValue(caps[slice]));
            ++configured;
        }
        std::cout << "RLC_CAP slice=" << slice << " rnti=" << rnti
                  << " drbs=" << configured << " bytes=" << caps[slice] << std::endl;
    }
}

class FilePolicyBridge
{
  public:
    FilePolicyBridge(const std::string& directory, const std::string& logFilename)
        : m_directory(directory)
    {
        NS_ABORT_MSG_IF(!std::filesystem::is_directory(m_directory),
                        "Policy bridge directory must already exist: " << directory);
        m_out.open(logFilename, std::ofstream::out | std::ofstream::trunc);
        NS_ABORT_MSG_IF(!m_out.is_open(), "Cannot open policy bridge action log");
        m_out << "decision_time_s,raw_urllc,raw_embb,raw_mmtc,raw_idle,"
                 "quota_urllc,quota_embb,quota_mmtc\n"
              << std::fixed << std::setprecision(9);
    }

    std::pair<std::array<double, 3>, std::array<double, 3>> Decide(
        Time decisionTime,
        const std::vector<double>& observation)
    {
        ++m_requestIndex;
        std::ostringstream label;
        label << std::setw(4) << std::setfill('0') << m_requestIndex;
        const auto request = m_directory / ("request_" + label.str() + ".csv");
        const auto temporary = m_directory / ("request_" + label.str() + ".tmp");
        const auto response = m_directory / ("response_" + label.str() + ".csv");
        const auto error = m_directory / ("error_" + label.str() + ".txt");
        NS_ABORT_MSG_IF(std::filesystem::exists(request) || std::filesystem::exists(response),
                        "Stale policy bridge files; use a fresh run directory");
        {
            std::ofstream out(temporary);
            NS_ABORT_MSG_IF(!out.is_open(), "Cannot write policy bridge request");
            out << std::setprecision(17) << decisionTime.GetSeconds();
            for (double value : observation)
            {
                out << ',' << value;
            }
            out << '\n';
        }
        std::filesystem::rename(temporary, request);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
        while (!std::filesystem::exists(response))
        {
            if (std::filesystem::exists(error))
            {
                std::ifstream failure(error);
                std::string message;
                std::getline(failure, message);
                NS_ABORT_MSG("Policy worker rejected request: " << message);
            }
            NS_ABORT_MSG_IF(std::chrono::steady_clock::now() >= deadline,
                            "Policy worker did not answer within 120 s");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        std::ifstream in(response);
        std::array<double, 3> raw{};
        int echoedIndex = 0;
        char comma = 0;
        in >> echoedIndex >> comma >> raw[0] >> comma >> raw[1] >> comma >> raw[2];
        NS_ABORT_MSG_IF(!in.good() && !in.eof(), "Malformed policy response");
        NS_ABORT_MSG_IF(echoedIndex != m_requestIndex ||
                            !std::all_of(raw.begin(), raw.end(), [](double value) {
                                return std::isfinite(value) && value >= 0;
                            }),
                        "Policy response identity/action is invalid");
        const double rawTotal = raw[0] + raw[1] + raw[2];
        NS_ABORT_MSG_IF(rawTotal > 1.0 + 1e-6, "Policy action exceeds unit resource budget");
        m_out << decisionTime.GetSeconds();
        for (double value : raw)
        {
            m_out << ',' << value;
        }
        m_out << ',' << 1.0 - rawTotal;
        for (double value : raw)
        {
            m_out << ',' << value;
        }
        m_out << '\n';
        m_out.flush();
        return {raw, raw};
    }

  private:
    std::filesystem::path m_directory;
    std::ofstream m_out;
    int m_requestIndex{0};
};

class ShareController
{
  public:
    ShareController(Ptr<BurstShieldShareScheduler> scheduler,
                    const std::string& filename,
                    const std::string& mode,
                    Time interval,
                    Time stop,
                    double stressStartS,
                    double stressEndS,
                    std::function<std::array<double, 3>(Time)> reactivePolicy)
        : m_scheduler(scheduler),
          m_mode(mode),
          m_interval(interval),
          m_stop(stop),
          m_stressStartS(stressStartS),
          m_stressEndS(stressEndS),
          m_reactivePolicy(std::move(reactivePolicy))
    {
        m_out.open(filename, std::ofstream::out | std::ofstream::trunc);
        NS_ABORT_MSG_IF(!m_out.is_open(), "Cannot open share-action CSV: " << filename);
        m_out << "decision_time_s,mode,previous_requested_urllc,previous_requested_embb,"
                 "previous_requested_mmtc,available_dl_symbols,actual_urllc_symbols,actual_embb_symbols,"
                 "actual_mmtc_symbols,next_requested_urllc,next_requested_embb,"
                 "next_requested_mmtc\n"
              << std::fixed << std::setprecision(6);
    }

    void Tick()
    {
        const double nowS = Simulator::Now().GetSeconds();
        const auto actual = m_scheduler->GetEpochSymbols();
        const bool stress = nowS >= m_stressStartS && nowS < m_stressEndS;
        const std::array<double, 3> next =
            (m_mode == "reactive" || m_mode == "pd" || m_mode == "external") ?
                m_reactivePolicy(Simulator::Now())
            : (m_mode == "step" && stress) ? std::array<double, 3>{0.70, 0.20, 0.10}
                                           : std::array<double, 3>{1.0 / 3.0, 1.0 / 3.0,
                                                                    1.0 / 3.0};
        m_out << nowS << ',' << m_mode;
        for (double share : m_currentShares)
        {
            m_out << ',' << share;
        }
        m_out << ',' << m_scheduler->GetEpochAvailableSymbols();
        for (uint64_t symbols : actual)
        {
            m_out << ',' << symbols;
        }
        for (double share : next)
        {
            m_out << ',' << share;
        }
        m_out << '\n';
        m_out.flush();
        m_scheduler->SetShares(next);
        m_currentShares = next;
        if (Simulator::Now() + m_interval <= m_stop)
        {
            Simulator::Schedule(m_interval, &ShareController::Tick, this);
        }
    }

  private:
    Ptr<BurstShieldShareScheduler> m_scheduler;
    std::ofstream m_out;
    std::string m_mode;
    Time m_interval;
    Time m_stop;
    double m_stressStartS;
    double m_stressEndS;
    std::function<std::array<double, 3>(Time)> m_reactivePolicy;
    std::array<double, 3> m_currentShares{1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
};

struct FlowCounters
{
    uint64_t txPackets = 0;
    uint64_t rxPackets = 0;
    Time delaySum = Seconds(0);
};

struct WindowTotals
{
    uint64_t txPackets = 0;
    uint64_t rxPackets = 0;
    double delaySumMs = 0.0;
};

struct PacketEntry
{
    uint16_t port = 0;
    Time sent = Seconds(0);
    Time deadline = Seconds(0);
    Time receivedAt = Seconds(0);
    uint32_t payloadBytes = 0;
    bool received = false;
    bool overflowed = false;
};

struct CohortKpi
{
    uint64_t offered = 0;
    uint64_t timely = 0;
    uint64_t overflow = 0;
    uint64_t timelyBits = 0;
    double timelyGoodputBps = 0.0;
    Time windowEnd = Seconds(0);
    bool available = false;
};

// A KPI window is defined by packet arrivals in that window. Tx and Rx counts
// are reported separately: their difference is NOT a packet-loss estimate.
class WindowRecorder
{
  public:
    WindowRecorder(Ptr<FlowMonitor> monitor,
                   Ptr<Ipv4FlowClassifier> classifier,
                   const std::string& filename,
                   const std::string& packetFilename,
                   const std::string& observationFilename,
                   const std::string& cohortFilename,
                   const std::string& deadlinePacketFilename,
                   Time window,
                   Time stop,
                   Time appStart,
                   uint16_t lowLatPort,
                   uint16_t voicePort,
                   uint16_t mmtcPort,
                   double lowLatSlaMs,
                   double voiceSlaMs,
                   double mmtcSlaMs,
                   double burstStartS,
                   double burstEndS,
                   double impairmentStartS,
                   double impairmentEndS,
                   std::vector<std::pair<double, double>> stressEvents,
                   bool customStress,
                   bool eventTraffic,
                   bool eventCapacity,
                   bool changedSlaProfile)
        : m_monitor(monitor),
          m_classifier(classifier),
          m_window(window),
          m_stop(stop),
          m_previousCohortEnd(appStart),
          m_lowLatPort(lowLatPort),
          m_voicePort(voicePort),
          m_mmtcPort(mmtcPort),
          m_lowLatSlaMs(lowLatSlaMs),
          m_voiceSlaMs(voiceSlaMs),
          m_mmtcSlaMs(mmtcSlaMs),
          m_burstStartS(burstStartS),
          m_burstEndS(burstEndS),
          m_impairmentStartS(impairmentStartS),
          m_impairmentEndS(impairmentEndS),
          m_stressEvents(std::move(stressEvents)),
          m_customStress(customStress),
          m_eventTraffic(eventTraffic),
          m_eventCapacity(eventCapacity),
          m_changedSlaProfile(changedSlaProfile)
    {
        m_out.open(filename, std::ofstream::out | std::ofstream::trunc);
        NS_ABORT_MSG_IF(!m_out.is_open(), "Cannot open KPI CSV: " << filename);
        m_out << "window_end_s,phase,flow,port,tx_packets,rx_packets,mean_rx_delay_ms,"
                 "sla_threshold_ms,violation,max_streak_windows\n";
        m_out << std::fixed << std::setprecision(6);

        m_packetOut.open(packetFilename, std::ofstream::out | std::ofstream::trunc);
        NS_ABORT_MSG_IF(!m_packetOut.is_open(), "Cannot open packet CSV: " << packetFilename);
        m_packetOut << "rx_time_s,flow,port,sequence,tx_time_s,one_way_delay_ms\n";
        m_packetOut << std::fixed << std::setprecision(6);

        m_observationOut.open(observationFilename, std::ofstream::out | std::ofstream::trunc);
        NS_ABORT_MSG_IF(!m_observationOut.is_open(),
                        "Cannot open control-observation CSV: " << observationFilename);
        m_observationOut << "decision_time_s";
        for (const auto& name : {"urllc", "embb", "mmtc"})
        {
            m_observationOut << ',' << name << "_past_tx_packets," << name
                             << "_past_rx_packets," << name << "_past_mean_rx_delay_ms," << name
                             << "_past_violation," << name << "_current_streak_windows," << name
                             << "_last_1s_deadline_reliability," << name
                             << "_last_1s_timely_goodput_bps," << name
                             << "_last_1s_cohort_end_s";
        }
        m_observationOut << '\n' << std::fixed << std::setprecision(6);

        m_cohortOut.open(cohortFilename, std::ofstream::out | std::ofstream::trunc);
        NS_ABORT_MSG_IF(!m_cohortOut.is_open(), "Cannot open cohort CSV: " << cohortFilename);
        m_cohortOut << "window_start_s,window_end_s,flow,port,offered_by_deadline,"
                       "timely_packets,missed_deadline_packets,reliability,"
                       "timely_goodput_bps,reliability_target,goodput_target_bps,"
                       "partial_compliant,warmup,load_adjusted_compliant\n"
                    << std::fixed << std::setprecision(9);

        m_deadlinePacketOut.open(deadlinePacketFilename, std::ofstream::out | std::ofstream::trunc);
        NS_ABORT_MSG_IF(!m_deadlinePacketOut.is_open(), "Cannot open deadline packet CSV: "
                                                       << deadlinePacketFilename);
        m_deadlinePacketOut << "window_start_s,window_end_s,flow,port,sequence,tx_time_s,"
                              "deadline_s,rx_time_at_closure_s,payload_bytes,timely\n"
                            << std::fixed << std::setprecision(9);
    }

    void OnLowLatRx(Ptr<const Packet> packet)
    {
        RecordPacket(packet, "URLLC", m_lowLatPort);
    }

    void OnVoiceRx(Ptr<const Packet> packet)
    {
        RecordPacket(packet, "eMBB", m_voicePort);
    }

    void OnMmtcRx(Ptr<const Packet> packet)
    {
        RecordPacket(packet, "mMTC", m_mmtcPort);
    }

    void OnLowLatRlcDrop(Ptr<const Packet> packet)
    {
        RecordRlcOverflow(packet, m_lowLatPort);
    }

    void OnVoiceRlcDrop(Ptr<const Packet> packet)
    {
        RecordRlcOverflow(packet, m_voicePort);
    }

    void OnMmtcRlcDrop(Ptr<const Packet> packet)
    {
        RecordRlcOverflow(packet, m_mmtcPort);
    }

    void OnIpTx(const Ipv4Header& ipHeader,
                Ptr<const Packet> packet,
                [[maybe_unused]] uint32_t interface)
    {
        if (ipHeader.GetProtocol() != 17)
        {
            return;
        }
        Ptr<Packet> copy = packet->Copy();
        UdpHeader udp;
        copy->RemoveHeader(udp);
        const uint16_t port = udp.GetDestinationPort();
        if (port != m_lowLatPort && port != m_voicePort && port != m_mmtcPort)
        {
            return;
        }
        SeqTsHeader header;
        NS_ABORT_MSG_IF(copy->RemoveHeader(header) != header.GetSerializedSize(),
                        "Expected UdpClient sequence/timestamp after UDP header");
        const double deadlineMs = port == m_lowLatPort ? m_lowLatSlaMs
                                  : port == m_voicePort ? m_voiceSlaMs
                                                        : m_mmtcSlaMs;
        const uint64_t key = PacketKey(port, header.GetSeq());
        const auto inserted = m_packetLedger.emplace(
            key,
            PacketEntry{port,
                        header.GetTs(),
                        header.GetTs() + Seconds(deadlineMs / 1000.0),
                        Seconds(0),
                        static_cast<uint32_t>(copy->GetSize() + header.GetSerializedSize()),
                        false,
                        false});
        NS_ABORT_MSG_IF(!inserted.second, "Duplicate sent UDP sequence for a slice");
        m_epochOfferedBits[port] += static_cast<double>(copy->GetSize() +
                                                        header.GetSerializedSize()) * 8.0;
    }

    void SampleCohort()
    {
        // The event runs one nanosecond after the boundary so packets delivered
        // exactly at their deadline are visible before the cohort is closed.
        const Time end = Simulator::Now() - NanoSeconds(1);
        std::map<uint16_t, CohortKpi> totals;
        for (const auto& item : m_packetLedger)
        {
            const PacketEntry& packet = item.second;
            if (packet.deadline <= m_previousCohortEnd || packet.deadline > end)
            {
                continue;
            }
            auto& kpi = totals[packet.port];
            ++kpi.offered;
            const bool timely = packet.received && packet.receivedAt <= packet.deadline;
            if (timely)
            {
                ++kpi.timely;
                kpi.timelyBits += static_cast<uint64_t>(packet.payloadBytes) * 8;
            }
            if (packet.overflowed)
            {
                ++kpi.overflow;
            }
            const char* flow = packet.port == m_lowLatPort ? "URLLC"
                               : packet.port == m_voicePort ? "eMBB"
                                                            : "mMTC";
            m_deadlinePacketOut << m_previousCohortEnd.GetSeconds() << ',' << end.GetSeconds()
                                << ',' << flow << ',' << packet.port << ','
                                << static_cast<uint32_t>(item.first & 0xffffffffu) << ','
                                << packet.sent.GetSeconds() << ',' << packet.deadline.GetSeconds()
                                << ',';
            if (packet.received)
            {
                m_deadlinePacketOut << packet.receivedAt.GetSeconds();
            }
            m_deadlinePacketOut << ',' << packet.payloadBytes << ',' << timely << '\n';
        }
        WriteCohortRow(end, "URLLC", m_lowLatPort,
                       m_changedSlaProfile ? 0.995 : 0.999,
                       (m_changedSlaProfile ? 0.93 : 0.95) * 256 * 8 * 400,
                       totals[m_lowLatPort]);
        WriteCohortRow(end, "eMBB", m_voicePort,
                       m_changedSlaProfile ? 0.995 : 0.990,
                       (m_changedSlaProfile ? 0.93 : 0.90) * 1200 * 8 * 600,
                       totals[m_voicePort]);
        WriteCohortRow(end, "mMTC", m_mmtcPort,
                       m_changedSlaProfile ? 0.990 : 0.980,
                       (m_changedSlaProfile ? 0.85 : 0.80) * 200 * 8 * 200,
                       totals[m_mmtcPort]);
        m_cohortOut.flush();
        m_deadlinePacketOut.flush();
        m_previousCohortEnd = end;
        m_firstCohort = false;
        if (Simulator::Now() + Seconds(1) <= m_stop)
        {
            Simulator::Schedule(Seconds(1), &WindowRecorder::SampleCohort, this);
        }
    }

    std::array<double, 3> ReactiveShares(Time decisionTime)
    {
        NS_ABORT_MSG_IF(!m_hasLatestSample || m_latestSampleTime != decisionTime,
                        "Reactive policy must use the completed window at this boundary");
        if (m_lastViolation.at(m_lowLatPort) == 1)
        {
            return {0.70, 0.20, 0.10};
        }
        if (m_lastViolation.at(m_voicePort) == 1)
        {
            return {0.25, 0.65, 0.10};
        }
        return {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
    }

    std::array<double, 3> GetLastWindowRxBits() const
    {
        return m_lastWindowRxBits;
    }

    // Transfer adapter: every entry is causal and derived from ns-3 telemetry,
    // but capacity and channel efficiency are proxies for the training simulator.
    std::vector<double> PolicyObservation(Time decisionTime,
                                          Ptr<BurstShieldShareScheduler> scheduler,
                                          bool telemetryV2) const
    {
        NS_ABORT_MSG_IF(!m_hasLatestSample || m_latestSampleTime != decisionTime,
                        "PD request requires the completed 100 ms observation");
        const std::array<uint16_t, 3> ports{m_lowLatPort, m_voicePort, m_mmtcPort};
        const auto queue = scheduler->GetQueueBits();
        const auto oldest = scheduler->GetOldestAgeMs();
        const auto mcs = scheduler->GetLastMcs();
        const double maxMcs = scheduler->GetMaxMcsSeen();
        const double intervalS = m_window.GetSeconds();
        // V2 observes a pre-allocation, full-slot PHY opportunity derived
        // from current CQI/MCS. It is still only a proxy for the exogenous
        // capacity field of the training simulator, but is not Rx throughput.
        const double capacityProxy = telemetryV2 ?
                                         scheduler->GetEpochPotentialBits() / intervalS :
                                         (m_lastWindowRxBits[0] + m_lastWindowRxBits[1] +
                                          m_lastWindowRxBits[2]) / intervalS;
        const double elapsed = (decisionTime - m_previousCohortEnd).GetSeconds();
        std::vector<double> result{capacityProxy, std::clamp(elapsed, 0.0, 1.0)};
        auto append = [&result](const std::array<double, 3>& values) {
            result.insert(result.end(), values.begin(), values.end());
        };
        append(queue);
        append(oldest);
        std::array<double, 3> arrival{}, efficiency{}, reliability{}, overflow{}, goodput{},
            active{}, streak{}, previous{}, deadline{10.0, 100.0, 500.0},
            reliabilityMin{0.999, 0.990, 0.980}, overflowMax{0.001, 0.010, 0.020},
            nominalGoodput{778240.0, 5184000.0, 256000.0}, partialOffered{},
            partialTimely{}, partialOverflow{}, partialFailed{};
        if (m_changedSlaProfile)
        {
            reliabilityMin = {0.995, 0.995, 0.990};
            overflowMax = {0.002, 0.005, 0.010};
            nominalGoodput = {761856.0, 5356800.0, 272000.0};
        }
        for (int index = 0; index < 3; ++index)
        {
            const uint16_t port = ports[index];
            arrival[index] = m_arrivalEwmaBits.at(port);
            efficiency[index] = maxMcs > 0 ? mcs[index] / maxMcs :
                                                  std::numeric_limits<double>::quiet_NaN();
            const auto cohort = m_lastCohort.find(port);
            if (cohort == m_lastCohort.end() || cohort->second.offered == 0)
            {
                reliability[index] = std::numeric_limits<double>::quiet_NaN();
                overflow[index] = std::numeric_limits<double>::quiet_NaN();
                goodput[index] = std::numeric_limits<double>::quiet_NaN();
            }
            else
            {
                reliability[index] = static_cast<double>(cohort->second.timely) /
                                     cohort->second.offered;
                overflow[index] = telemetryV2 ?
                                      static_cast<double>(cohort->second.overflow) /
                                          cohort->second.offered :
                                      std::numeric_limits<double>::quiet_NaN();
                goodput[index] = cohort->second.timelyGoodputBps;
                active[index] = 1.0;
            }
            const auto streakEntry = m_kpiStreak.find(port);
            streak[index] = streakEntry == m_kpiStreak.end() ? 0.0 : streakEntry->second;
            previous[index] = m_previousPolicyAction[index];
        }
        for (const auto& item : m_packetLedger)
        {
            const PacketEntry& packet = item.second;
            if (packet.deadline <= m_previousCohortEnd || packet.deadline > decisionTime)
            {
                continue;
            }
            const int index = packet.port == m_lowLatPort ? 0
                              : packet.port == m_voicePort ? 1
                                                           : 2;
            ++partialOffered[index];
            if (packet.received && packet.receivedAt <= packet.deadline)
            {
                ++partialTimely[index];
            }
            else
            {
                ++partialFailed[index];
            }
            if (packet.overflowed)
            {
                ++partialOverflow[index];
            }
        }
        append(arrival);
        append(efficiency);
        append(reliability);
        append(overflow);
        append(goodput);
        append(active);
        append(streak);
        append(previous);
        append(deadline);
        append(reliabilityMin);
        append(overflowMax);
        append(nominalGoodput);
        append(partialOffered);
        append(partialTimely);
        if (!telemetryV2)
        {
            partialOverflow.fill(std::numeric_limits<double>::quiet_NaN());
        }
        append(partialOverflow);
        append(partialFailed);
        NS_ABORT_MSG_IF(result.size() != 56, "PD raw causal feature count is not 56");
        result.push_back(m_previousCohortEnd.GetSeconds());
        for (uint16_t port : ports)
        {
            const auto cohort = m_lastCohort.find(port);
            result.push_back(cohort == m_lastCohort.end() ? 0.0 : cohort->second.offered);
        }
        return result;
    }

    void SetPreviousPolicyAction(const std::array<double, 3>& action)
    {
        m_previousPolicyAction = action;
    }

    void Sample()
    {
        std::map<uint16_t, WindowTotals> totals;
        const auto stats = m_monitor->GetFlowStats();
        for (const auto& entry : stats)
        {
            const auto tuple = m_classifier->FindFlow(entry.first);
            if (tuple.destinationPort != m_lowLatPort && tuple.destinationPort != m_voicePort &&
                tuple.destinationPort != m_mmtcPort)
            {
                continue;
            }

            const FlowCounters current{entry.second.txPackets,
                                       entry.second.rxPackets,
                                       entry.second.delaySum};
            const auto previous = m_previous[entry.first];
            auto& window = totals[tuple.destinationPort];
            window.txPackets += current.txPackets - previous.txPackets;
            window.rxPackets += current.rxPackets - previous.rxPackets;
            window.delaySumMs +=
                (current.delaySum - previous.delaySum).GetSeconds() * 1000.0;
            m_previous[entry.first] = current;
        }

        const double nowS = Simulator::Now().GetSeconds();
        WriteRow(nowS, "URLLC", m_lowLatPort, m_lowLatSlaMs, totals[m_lowLatPort]);
        WriteRow(nowS, "eMBB", m_voicePort, m_voiceSlaMs, totals[m_voicePort]);
        WriteRow(nowS, "mMTC", m_mmtcPort, m_mmtcSlaMs, totals[m_mmtcPort]);
        m_lastWindowRxBits = {
            static_cast<double>(totals[m_lowLatPort].rxPackets) * 256.0 * 8.0,
            static_cast<double>(totals[m_voicePort].rxPackets) * 1200.0 * 8.0,
            static_cast<double>(totals[m_mmtcPort].rxPackets) * 200.0 * 8.0};
        m_out.flush();

        // Snapshot only the just-completed window. This is a causal telemetry
        // boundary for a future controller, not a scheduler actuation.
        m_observationOut << nowS;
        for (uint16_t port : {m_lowLatPort, m_voicePort, m_mmtcPort})
        {
            const auto& window = totals[port];
            m_observationOut << ',' << window.txPackets << ',' << window.rxPackets << ',';
            if (window.rxPackets > 0)
            {
                m_observationOut << window.delaySumMs / window.rxPackets;
            }
            m_observationOut << ',' << m_lastViolation[port] << ',' << m_currentStreak[port];
            const auto cohort = m_lastCohort.find(port);
            if (cohort == m_lastCohort.end())
            {
                m_observationOut << ",,,";
            }
            else
            {
                m_observationOut << ',';
                if (cohort->second.offered > 0)
                {
                    m_observationOut << static_cast<double>(cohort->second.timely) /
                                            cohort->second.offered;
                }
                m_observationOut << ',' << cohort->second.timelyGoodputBps << ','
                                 << cohort->second.windowEnd.GetSeconds();
            }
        }
        m_observationOut << '\n';
        m_observationOut.flush();
        m_latestSampleTime = Simulator::Now();
        m_hasLatestSample = true;
        for (uint16_t port : {m_lowLatPort, m_voicePort, m_mmtcPort})
        {
            m_arrivalEwmaBits[port] =
                0.9 * m_arrivalEwmaBits[port] + 0.1 * m_epochOfferedBits[port];
            m_epochOfferedBits[port] = 0.0;
        }

        if (Simulator::Now() + m_window <= m_stop)
        {
            Simulator::Schedule(m_window, &WindowRecorder::Sample, this);
        }
    }

  private:
    static uint64_t PacketKey(uint16_t port, uint32_t sequence)
    {
        return (static_cast<uint64_t>(port) << 32) | sequence;
    }

    void RecordRlcOverflow(Ptr<const Packet> packet, uint16_t expectedPort)
    {
        if (DeadlineDropInProgress()) return; // Expiry remains a deadline miss, not buffer overflow.
        Ptr<Packet> copy = packet->Copy();
        NrPdcpHeader pdcp;
        copy->RemoveHeader(pdcp);
        uint8_t firstByte = 0;
        NS_ABORT_MSG_IF(copy->GetSize() < 1 || copy->CopyData(&firstByte, 1) != 1 ||
                            (firstByte >> 4) != 4,
                        "RLC overflow PDU does not contain an IPv4 datagram");
        Ipv4Header ip;
        copy->RemoveHeader(ip);
        NS_ABORT_MSG_IF(ip.GetProtocol() != 17, "Expected UDP inside dropped RLC PDU");
        UdpHeader udp;
        copy->RemoveHeader(udp);
        NS_ABORT_MSG_IF(udp.GetDestinationPort() != expectedPort,
                        "RLC overflow PDU belongs to the wrong slice");
        SeqTsHeader header;
        copy->RemoveHeader(header);
        auto entry = m_packetLedger.find(PacketKey(expectedPort, header.GetSeq()));
        NS_ABORT_MSG_IF(entry == m_packetLedger.end() || entry->second.overflowed,
                        "RLC overflow is not a unique application packet");
        entry->second.overflowed = true;
    }

    void RecordPacket(Ptr<const Packet> packet, const std::string& flow, uint16_t port)
    {
        SeqTsHeader header;
        NS_ABORT_MSG_IF(packet->PeekHeader(header) != header.GetSerializedSize(),
                        "Expected UdpClient sequence/timestamp header");
        const auto entry = m_packetLedger.find(PacketKey(port, header.GetSeq()));
        NS_ABORT_MSG_IF(entry == m_packetLedger.end() || entry->second.received,
                        "Received UDP sequence lacks a unique sent-ledger entry");
        entry->second.received = true;
        entry->second.receivedAt = Simulator::Now();
        m_packetOut << Simulator::Now().GetSeconds() << ',' << flow << ',' << port << ','
                    << header.GetSeq() << ',' << header.GetTs().GetSeconds() << ','
                    << (Simulator::Now() - header.GetTs()).GetSeconds() * 1000.0 << '\n';
    }

    void WriteCohortRow(Time end,
                        const std::string& flow,
                        uint16_t port,
                        double reliabilityTarget,
                        double goodputTargetBps,
                        CohortKpi kpi)
    {
        kpi.windowEnd = end;
        kpi.available = true;
        const double intervalS = (end - m_previousCohortEnd).GetSeconds();
        const double goodput = kpi.timelyBits / intervalS;
        kpi.timelyGoodputBps = goodput;
        m_lastCohort[port] = kpi;
        const double offeredGoodputBps = static_cast<double>(kpi.offered) *
                                            (port == m_lowLatPort ? 256.0 :
                                             port == m_voicePort ? 1200.0 : 200.0) *
                                            8.0 / intervalS;
        const auto compliance = deadline_cohort::Evaluate(kpi.offered,
                                                         kpi.timely,
                                                         goodput,
                                                         reliabilityTarget,
                                                         goodputTargetBps,
                                                         offeredGoodputBps,
                                                         m_kpiStreak[port],
                                                         m_firstCohort);
        m_kpiStreak[port] = compliance.nextStreak;
        m_cohortOut << m_previousCohortEnd.GetSeconds() << ',' << end.GetSeconds() << ','
                    << flow << ',' << port << ',' << kpi.offered << ',' << kpi.timely << ','
                    << kpi.offered - kpi.timely << ',';
        if (kpi.offered > 0)
        {
            m_cohortOut << static_cast<double>(kpi.timely) / kpi.offered;
        }
        m_cohortOut << ',' << goodput << ',' << reliabilityTarget << ',' << goodputTargetBps << ',';
        // Keep legacy partial_compliant as the uncapped nominal-goodput SLA.
        // New flag uses native strict comparisons; H shares the target, not tolerances.
        m_cohortOut << compliance.nominalCompliant;
        m_cohortOut << ',' << (m_firstCohort ? 1 : 0) << ','
                    << compliance.loadAdjustedCompliant << '\n';
    }

    std::string Phase(double nowS) const
    {
        if (m_customStress)
        {
            const bool active = std::any_of(m_stressEvents.begin(), m_stressEvents.end(),
                                            [nowS](const auto& event) {
                                                return nowS > event.first &&
                                                       nowS <= event.second;
                                            });
            if (active)
            {
                return m_eventTraffic && m_eventCapacity ? "compound" :
                       m_eventTraffic ? "traffic_burst" : "capacity_loss";
            }
            return !m_stressEvents.empty() && nowS > m_stressEvents.back().second ?
                       "recovery" : "nominal";
        }
        const bool burst = nowS > m_burstStartS && nowS <= m_burstEndS;
        const bool impairment = nowS > m_impairmentStartS && nowS <= m_impairmentEndS;
        if (burst && impairment)
        {
            return "compound";
        }
        if (burst)
        {
            return "traffic_burst";
        }
        if (impairment)
        {
            return "radio_impairment";
        }
        if (nowS > std::max(m_burstEndS, m_impairmentEndS))
        {
            return "recovery";
        }
        return "nominal";
    }

    void WriteRow(double nowS,
                  const std::string& flow,
                  uint16_t port,
                  double slaMs,
                  const WindowTotals& totals)
    {
        // -1 means the window has no traffic observation and is not scored.
        int violation = -1;
        if (totals.rxPackets > 0)
        {
            violation = (totals.delaySumMs / totals.rxPackets > slaMs) ? 1 : 0;
        }
        else if (totals.txPackets > 0)
        {
            violation = 1;
        }
        m_lastViolation[port] = violation;

        if (violation == 1)
        {
            ++m_currentStreak[port];
            m_maxStreak[port] = std::max(m_maxStreak[port], m_currentStreak[port]);
        }
        else if (violation == 0)
        {
            m_currentStreak[port] = 0;
        }

        m_out << nowS << ',' << Phase(nowS) << ',' << flow << ',' << port << ','
              << totals.txPackets << ',' << totals.rxPackets << ',';
        if (totals.rxPackets > 0)
        {
            m_out << totals.delaySumMs / totals.rxPackets;
        }
        m_out << ',' << slaMs << ',' << violation << ',' << m_maxStreak[port] << '\n';
    }

    Ptr<FlowMonitor> m_monitor;
    Ptr<Ipv4FlowClassifier> m_classifier;
    std::ofstream m_out;
    std::ofstream m_packetOut;
    std::ofstream m_observationOut;
    std::ofstream m_cohortOut;
    std::ofstream m_deadlinePacketOut;
    Time m_window;
    Time m_stop;
    Time m_previousCohortEnd;
    bool m_firstCohort{true};
    uint16_t m_lowLatPort;
    uint16_t m_voicePort;
    uint16_t m_mmtcPort;
    double m_lowLatSlaMs;
    double m_voiceSlaMs;
    double m_mmtcSlaMs;
    double m_burstStartS;
    double m_burstEndS;
    double m_impairmentStartS;
    double m_impairmentEndS;
    std::vector<std::pair<double, double>> m_stressEvents;
    bool m_customStress;
    bool m_eventTraffic;
    bool m_eventCapacity;
    bool m_changedSlaProfile;
    std::map<FlowId, FlowCounters> m_previous;
    std::map<uint16_t, uint32_t> m_currentStreak;
    std::map<uint16_t, uint32_t> m_maxStreak;
    std::map<uint16_t, int> m_lastViolation;
    std::map<uint64_t, PacketEntry> m_packetLedger;
    std::map<uint16_t, double> m_epochOfferedBits;
    std::map<uint16_t, double> m_arrivalEwmaBits;
    std::map<uint16_t, uint32_t> m_kpiStreak;
    std::array<double, 3> m_previousPolicyAction{};
    std::array<double, 3> m_lastWindowRxBits{};
    std::map<uint16_t, CohortKpi> m_lastCohort;
    Time m_latestSampleTime{Seconds(0)};
    bool m_hasLatestSample{false};
};

void
AttachRlcOverflowTraces(Ptr<NrGnbNetDevice> gnb,
                        const NetDeviceContainer& urllcDevices,
                        const NetDeviceContainer& embbDevices,
                        const NetDeviceContainer& mmtcDevices,
                        WindowRecorder* recorder)
{
    const std::array<Ptr<NrUeNetDevice>, 3> ues = {
        DynamicCast<NrUeNetDevice>(urllcDevices.Get(0)),
        DynamicCast<NrUeNetDevice>(embbDevices.Get(0)),
        DynamicCast<NrUeNetDevice>(mmtcDevices.Get(0))};
    for (size_t slice = 0; slice < ues.size(); ++slice)
    {
        const uint16_t rnti = ues[slice]->GetRrc()->GetRnti();
        Ptr<NrUeManager> manager = gnb->GetRrc()->GetUeManager(rnti);
        NS_ABORT_MSG_IF(!manager, "Missing UE manager for RLC overflow telemetry");
        ObjectMapValue bearers;
        manager->GetAttribute("DataRadioBearerMap", bearers);
        NS_ABORT_MSG_IF(bearers.GetN() < 2, "Expected default and dedicated DRBs");
        for (auto it = bearers.Begin(); it != bearers.End(); ++it)
        {
            Ptr<NrDataRadioBearerInfo> info = it->second->GetObject<NrDataRadioBearerInfo>();
            Ptr<NrRlcUm> rlc = DynamicCast<NrRlcUm>(info->m_rlc);
            NS_ABORT_MSG_IF(!rlc, "Overflow telemetry requires NR RLC UM");
            bool connected = false;
            if (slice == 0)
            {
                connected = rlc->TraceConnectWithoutContext(
                    "TxDrop", MakeCallback(&WindowRecorder::OnLowLatRlcDrop, recorder));
            }
            else if (slice == 1)
            {
                connected = rlc->TraceConnectWithoutContext(
                    "TxDrop", MakeCallback(&WindowRecorder::OnVoiceRlcDrop, recorder));
            }
            else
            {
                connected = rlc->TraceConnectWithoutContext(
                    "TxDrop", MakeCallback(&WindowRecorder::OnMmtcRlcDrop, recorder));
            }
            NS_ABORT_MSG_IF(!connected, "Cannot connect RLC TxDrop trace");
        }
    }
}

void
SetClientRate(const ApplicationContainer& apps, uint32_t packetsPerSecond)
{
    const Time interval = Seconds(1.0 / packetsPerSecond);
    for (uint32_t i = 0; i < apps.GetN(); ++i)
    {
        apps.Get(i)->SetAttribute("Interval", TimeValue(interval));
    }
}

void
SetRadioPower(Ptr<NrHelper> nrHelper, const NetDeviceContainer& gnbDevices, double powerDbm)
{
    for (uint32_t i = 0; i < gnbDevices.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(gnbDevices.Get(i), 0)->SetAttribute("TxPower", DoubleValue(powerDbm));
    }
}

} // namespace

int
FrozenMain(int argc, char* argv[])
{
    /*
     * Variables that represent the parameters we will accept as input by the
     * command line. Each of them is initialized with a default value, and
     * possibly overridden below when command-line arguments are parsed.
     */
    // Scenario parameters (that we will use inside this script):
    uint16_t gNbNum = 1;
    uint16_t ueNumPergNb = 3;
    bool logging = false;
    bool deadlineExpiry = false;
    bool fastDeadline = false;

    // Simulation parameters. Please don't use double to indicate seconds; use
    // ns-3 Time values which use integers to avoid portability issues.
    Time simTime = Seconds(5);
    Time udpAppStartTime = MilliSeconds(400);

    // NR parameters. We will take the input from the command line, and then we
    // will pass them inside the NR module.
    uint16_t numerology = 0;
    double centralFrequency = 4e9;
    double bandwidth = 5e6;
    double totalTxPower = 43;

    bool enableOfdma = false;

    uint16_t mcsTable = 2;

    // Nominal sizes/rates match DEFAULT_PROFILES in burstshield_packet/model.py;
    // traffic timing and bearer choices remain simplified calibration proxies.
    uint32_t lowLatRatePps = 400;
    uint32_t voiceRatePps = 600;
    uint32_t mmtcRatePps = 200;
    uint32_t lowLatPacketBytes = 256;
    uint32_t voicePacketBytes = 1200;
    uint32_t mmtcPacketBytes = 200;
    uint32_t burstFactor = 4;
    double burstStartS = 1.5;
    double burstEndS = 2.5;
    double impairmentStartS = 2.0;
    double impairmentEndS = 3.0;
    double radioPowerDropDb = 45.0;
    // Optional native 24-condition schedule. "nominal" disables the legacy
    // single burst/power-drop pair; otherwise supply start:end pairs separated
    // by commas. Capacity loss is an exogenous scheduler-symbol ceiling, not
    // a radio-power or PRB-bandwidth change.
    std::string stressEvents;
    std::string replaySchedule;
    double trafficScaleUrllc = 1.0;
    double trafficScaleEmbb = 1.0;
    double trafficScaleMmtc = 1.0;
    double capacityScale = 1.0;
    bool changedSlaProfile = false;
    double lowLatSlaMs = 10.0;
    double voiceSlaMs = 100.0;
    double mmtcSlaMs = 500.0;
    uint32_t kpiWindowMs = 100;
    uint32_t rlcMaxTxBufferBytes = 999999999;
    uint32_t rlcCapUrllcBytes = 0;
    uint32_t rlcCapEmbbBytes = 0;
    uint32_t rlcCapMmtcBytes = 0;
    bool telemetryV2 = false;
    uint32_t rngRun = 1;

    // Where we will store the output files.
    std::string simTag = "burstshield-stage7";
    std::string outputDir = "./";
    std::string bridgeDir;
    std::string shareMode = "equal";

    /*
     * From here, we instruct the ns3::CommandLine class of all the input parameters
     * that we may accept as input, as well as their description, and the storage
     * variable.
     */
    CommandLine cmd;
    cmd.AddValue("deadlineExpiry", "Expire complete unsent SDUs; all drops stay in offered denominator", deadlineExpiry);
    cmd.AddValue("fastDeadline", "Slot-level urgency order within the same quota ceilings", fastDeadline);

    cmd.AddValue("gNbNum", "The number of gNbs in multiple-ue topology", gNbNum);
    cmd.AddValue("ueNumPergNb", "The number of UE per gNb in multiple-ue topology", ueNumPergNb);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("numerology", "The numerology to be used", numerology);
    cmd.AddValue("centralFrequency", "The system frequency to be used", centralFrequency);
    cmd.AddValue("bandwidth", "The system bandwidth to be used", bandwidth);
    cmd.AddValue("totalTxPower",
                 "total tx power that will be proportionally assigned to"
                 " bands, CCs and bandwidth parts depending on each BWP bandwidth ",
                 totalTxPower);
    cmd.AddValue("simTag",
                 "tag to be appended to output filenames to distinguish simulation campaigns",
                 simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);
    cmd.AddValue("shareMode", "Actuator check: equal, step, reactive, pd or external", shareMode);
    cmd.AddValue("bridgeDir", "Fresh shared directory for external policy request/response", bridgeDir);
    cmd.AddValue("enableOfdma",
                 "If set to true it enables Ofdma scheduler. Default value is false (Tdma)",
                 enableOfdma);
    cmd.AddValue("lowLatRatePps", "Nominal URLLC-proxy packets per second", lowLatRatePps);
    cmd.AddValue("voiceRatePps", "Nominal eMBB-proxy packets per second", voiceRatePps);
    cmd.AddValue("mmtcRatePps", "Nominal mMTC-proxy packets per second", mmtcRatePps);
    cmd.AddValue("lowLatPacketBytes", "URLLC-proxy UDP payload bytes", lowLatPacketBytes);
    cmd.AddValue("voicePacketBytes", "eMBB-proxy UDP payload bytes", voicePacketBytes);
    cmd.AddValue("mmtcPacketBytes", "mMTC-proxy UDP payload bytes", mmtcPacketBytes);
    cmd.AddValue("burstFactor", "Integer multiplier for the URLLC-proxy rate", burstFactor);
    cmd.AddValue("burstStartS", "Traffic-burst onset in seconds", burstStartS);
    cmd.AddValue("burstEndS", "Traffic-burst end in seconds", burstEndS);
    cmd.AddValue("impairmentStartS", "Radio-impairment onset in seconds", impairmentStartS);
    cmd.AddValue("impairmentEndS", "Radio-impairment end in seconds", impairmentEndS);
    cmd.AddValue("radioPowerDropDb", "Temporary gNB transmit-power reduction in dB", radioPowerDropDb);
    cmd.AddValue("stressEvents", "Native stress schedule: nominal or comma-separated start:end seconds", stressEvents);
    cmd.AddValue("replaySchedule", "Exogenous CSV: time,u_pps,e_pps,m_pps,power_drop_db", replaySchedule);
    cmd.AddValue("trafficScaleUrllc", "URLLC rate multiplier during native stress events", trafficScaleUrllc);
    cmd.AddValue("trafficScaleEmbb", "eMBB rate multiplier during native stress events", trafficScaleEmbb);
    cmd.AddValue("trafficScaleMmtc", "mMTC rate multiplier during native stress events", trafficScaleMmtc);
    cmd.AddValue("capacityScale", "Fraction of DL scheduler symbols usable during native stress events", capacityScale);
    cmd.AddValue("changedSlaProfile", "Use the original matrix's changed reliability/goodput SLA profile", changedSlaProfile);
    cmd.AddValue("lowLatSlaMs", "Exploratory URLLC mean-delay threshold in ms", lowLatSlaMs);
    cmd.AddValue("voiceSlaMs", "Exploratory eMBB mean-delay threshold in ms", voiceSlaMs);
    cmd.AddValue("mmtcSlaMs", "Exploratory mMTC mean-delay threshold in ms", mmtcSlaMs);
    cmd.AddValue("kpiWindowMs", "KPI sampling window in milliseconds", kpiWindowMs);
    cmd.AddValue("rlcMaxTxBufferBytes", "Global RLC Tx-buffer cap in bytes (diagnostic)",
                 rlcMaxTxBufferBytes);
    cmd.AddValue("rlcCapUrllcBytes", "Optional gNB RLC UM cap for URLLC UE in bytes",
                 rlcCapUrllcBytes);
    cmd.AddValue("rlcCapEmbbBytes", "Optional gNB RLC UM cap for eMBB UE in bytes",
                 rlcCapEmbbBytes);
    cmd.AddValue("rlcCapMmtcBytes", "Optional gNB RLC UM cap for mMTC UE in bytes",
                 rlcCapMmtcBytes);
    cmd.AddValue("telemetryV2", "Use pre-allocation NR capacity and exact RLC overflow telemetry",
                 telemetryV2);
    cmd.AddValue("rngRun", "ns-3 random run number", rngRun);

    cmd.Parse(argc, argv);
    ns3::SetDeadlineExpiry(deadlineExpiry);
    ns3::gFastDeadline = fastDeadline;

    NS_ABORT_MSG_IF(gNbNum != 1 || ueNumPergNb != 3,
                    "Stage 3 is intentionally restricted to one gNB and three UEs");
    NS_ABORT_MSG_IF(shareMode != "equal" && shareMode != "step" &&
                        shareMode != "reactive" && shareMode != "pd" &&
                        shareMode != "external",
                    "shareMode must be equal, step, reactive, pd or external");
    NS_ABORT_MSG_IF((shareMode == "pd" || shareMode == "external") && bridgeDir.empty(),
                    "Policy bridge mode requires bridgeDir");
    NS_ABORT_MSG_IF(enableOfdma, "Stage-3 share actuator supports TDMA only");
    NS_ABORT_MSG_IF(lowLatRatePps == 0 || voiceRatePps == 0 || mmtcRatePps == 0 ||
                        burstFactor == 0,
                    "Packet rates and burstFactor must be positive");
    NS_ABORT_MSG_IF(lowLatRatePps > std::numeric_limits<uint32_t>::max() / burstFactor,
                    "Burst rate overflows uint32_t");
    NS_ABORT_MSG_IF(lowLatPacketBytes < 12 || voicePacketBytes < 12 ||
                        mmtcPacketBytes < 12 || lowLatPacketBytes > 65507 ||
                        voicePacketBytes > 65507 || mmtcPacketBytes > 65507,
                    "UDP payload sizes must be between 12 and 65507 bytes");
    NS_ABORT_MSG_IF(kpiWindowMs == 0 || lowLatSlaMs <= 0 || voiceSlaMs <= 0 ||
                        mmtcSlaMs <= 0,
                    "KPI window and SLA thresholds must be positive");
    NS_ABORT_MSG_IF(burstStartS <= udpAppStartTime.GetSeconds() ||
                        burstStartS >= burstEndS ||
                        impairmentStartS <= udpAppStartTime.GetSeconds() ||
                        impairmentStartS >= impairmentEndS ||
                        std::max(burstEndS, impairmentEndS) >= simTime.GetSeconds() ||
                        radioPowerDropDb < 0,
                        "Stress intervals must lie within the active simulation and power drop >= 0");
    const bool customStress = !stressEvents.empty();
    std::vector<std::pair<double, double>> nativeEvents;
    if (customStress && stressEvents != "nominal")
    {
        std::stringstream input(stressEvents);
        std::string item;
        while (std::getline(input, item, ','))
        {
            const auto colon = item.find(':');
            NS_ABORT_MSG_IF(colon == std::string::npos, "Malformed native stress event");
            const double start = std::stod(item.substr(0, colon));
            const double end = std::stod(item.substr(colon + 1));
            NS_ABORT_MSG_IF(!std::isfinite(start) || !std::isfinite(end) ||
                                start <= udpAppStartTime.GetSeconds() ||
                                end <= start || end >= simTime.GetSeconds() ||
                                (!nativeEvents.empty() && start <= nativeEvents.back().second),
                            "Native events must be ordered, disjoint and inside simulation");
            nativeEvents.emplace_back(start, end);
        }
        NS_ABORT_MSG_IF(nativeEvents.empty(), "Native stress schedule is empty");
    }
    const bool nativeTraffic = trafficScaleUrllc != 1.0 || trafficScaleEmbb != 1.0 ||
                               trafficScaleMmtc != 1.0;
    const bool nativeCapacity = capacityScale != 1.0;
    NS_ABORT_MSG_IF(customStress && (!std::isfinite(trafficScaleUrllc) ||
                        !std::isfinite(trafficScaleEmbb) || !std::isfinite(trafficScaleMmtc) ||
                        trafficScaleUrllc <= 0 || trafficScaleEmbb <= 0 ||
                        trafficScaleMmtc <= 0 || !std::isfinite(capacityScale) ||
                        capacityScale <= 0 || capacityScale > 1.0 ||
                        (stressEvents == "nominal" && (nativeTraffic || nativeCapacity)) ||
                        (stressEvents != "nominal" && !nativeTraffic && !nativeCapacity)),
                    "Invalid native stress multipliers");
    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(rngRun);

    // enable logging or not
    if (logging)
    {
        LogLevel logLevel1 =
            (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_INFO);
        LogComponentEnable("NrMacSchedulerNs3", logLevel1);
        LogComponentEnable("NrMacSchedulerTdma", logLevel1);
    }

    NS_ABORT_MSG_IF(rlcMaxTxBufferBytes == 0, "RLC Tx buffer cap must be positive");
    const bool perSliceRlcCaps = rlcCapUrllcBytes || rlcCapEmbbBytes || rlcCapMmtcBytes;
    NS_ABORT_MSG_IF(perSliceRlcCaps &&
                        (!rlcCapUrllcBytes || !rlcCapEmbbBytes || !rlcCapMmtcBytes),
                    "Specify all three per-slice RLC caps or none");
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(rlcMaxTxBufferBytes));
    if (telemetryV2)
    {
        // In this 5G-LENA build, the optional HOL 'discard' branch emits
        // TxDrop but still enqueues the packet. Disable that misleading trace;
        // real buffer-full discards still emit TxDrop and retain their behavior.
        Config::SetDefault("ns3::NrRlcUm::EnablePdcpDiscarding", BooleanValue(false));
    }

    /*
     * Create the scenario. In our examples, we heavily use helpers that setup
     * the gnbs and ue following a pre-defined pattern. Please have a look at the
     * GridScenarioHelper documentation to see how the nodes will be distributed.
     */
    int64_t randomStream = 1;

    GridScenarioHelper gridScenario;
    gridScenario.SetRows(1);
    gridScenario.SetColumns(gNbNum);
    gridScenario.SetHorizontalBsDistance(5.0);
    gridScenario.SetVerticalBsDistance(5.0);
    gridScenario.SetBsHeight(umi_geometry::kBsHeightM);
    gridScenario.SetUtHeight(1.5);
    // must be set before BS number
    gridScenario.SetSectorization(GridScenarioHelper::SINGLE);
    gridScenario.SetBsNumber(gNbNum);
    gridScenario.SetUtNumber(ueNumPergNb * gNbNum);
    gridScenario.SetScenarioHeight(3); // Create a 3x3 scenario where the UE will
    gridScenario.SetScenarioLength(3); // be distributed.
    randomStream += gridScenario.AssignStreams(randomStream);
    gridScenario.CreateScenario();

    // Preserve helper sampling and stream consumption. The only position
    // operation is a deterministic +20 m x translation before NR setup.
    NS_ABORT_MSG_IF(gridScenario.GetBaseStations().GetN() != 1 ||
                        gridScenario.GetUserTerminals().GetN() != 3,
                    "UMi fixed-model evaluation expects exactly one gNB and three UEs");
    const auto bsPosition = gridScenario.GetBaseStations().Get(0)
                                ->GetObject<MobilityModel>()->GetPosition();
    const umi_geometry::Position umiBs{bsPosition.x, bsPosition.y, bsPosition.z};
    std::ofstream geometryOut(outputDir + "/" + simTag + "_umi_geometry.json");
    NS_ABORT_MSG_IF(!geometryOut.is_open(), "Cannot create UMi geometry record");
    geometryOut << std::setprecision(17)
                << "{\"schema_version\":1,\"profile\":\"UMi_StreetCanyon_fixed20m\","
                << "\"rng_run\":" << rngRun
                << ",\"frequency_hz\":" << centralFrequency
                << ",\"bandwidth_hz\":" << bandwidth
                << ",\"deterministic_x_shift_m\":20,\"helper_extent_m\":[3,3],"
                << "\"gNB_position_m\":[" << bsPosition.x << ',' << bsPosition.y
                << ',' << bsPosition.z << "],\"ue_positions\":[";
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        auto mobility = gridScenario.GetUserTerminals().Get(j)->GetObject<MobilityModel>();
        const auto original = mobility->GetPosition();
        const auto corrected = umi_geometry::Translate({original.x, original.y, original.z});
        umi_geometry::Check(umiBs, corrected);
        mobility->SetPosition(Vector(corrected.x, corrected.y, corrected.z));
        geometryOut << (j ? "," : "") << "{\"ue_index\":" << j
                    << ",\"original_position_m\":[" << original.x << ',' << original.y
                    << ',' << original.z << "],\"position_m\":[" << corrected.x << ','
                    << corrected.y << ',' << corrected.z << "],\"d2d_m\":"
                    << umi_geometry::Distance2D(umiBs, corrected) << '}';
    }
    geometryOut << "]}\n";
    geometryOut.close();
    NS_ABORT_MSG_IF(!geometryOut, "Failed writing UMi geometry record");

    /*
     * The three UEs carry one traffic class each on a shared BWP.
     */
    NodeContainer ueLowLatContainer;
    NodeContainer ueVoiceContainer;
    NodeContainer ueMmtcContainer;

    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Node> ue = gridScenario.GetUserTerminals().Get(j);

        if (j == 0)
        {
            ueLowLatContainer.Add(ue);
        }
        else if (j == 1)
        {
            ueVoiceContainer.Add(ue);
        }
        else
        {
            ueMmtcContainer.Add(ue);
        }
    }

    // setup the nr simulation
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));

    std::cout << "SchedulerType: ns3::BurstShieldShareScheduler; shareMode: " << shareMode
              << std::endl;
    nrHelper->SetSchedulerTypeId(BurstShieldShareScheduler::GetTypeId());

    // Error Model: gNB and UE with same spectrum error model.
    std::string errorModel = "ns3::NrEesmIrT" + std::to_string(mcsTable);
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetUlErrorModel(errorModel);

    // Both DL and UL AMC will have the same model behind.
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    // Beamforming method
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Antennas for all the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for all the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    /*
     * Setup the configuration of the spectrum. One operation band is deployed
     * with 1 component carrier (CC), automatically generated by the ccBwpManager
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    OperationBandInfo band;
    const uint8_t numOfCcs = 1;

    /*
     * The configured spectrum division for TDD is:
     *
     * |----Band1----|
     * |-----CC1-----|
     * |-----BWP1----|
     */

    // Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
    // a single BWP per CC
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,
                                                   bandwidth,
                                                   numOfCcs,
                                                   BandwidthPartInfo::UMi_StreetCanyon);

    bandConf.m_numBwp = 1;
    // By using the configuration created, it is time to make the operation band
    band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    double x = pow(10, totalTxPower / 10);

    Packet::EnableChecking();
    Packet::EnablePrinting();

    uint32_t bwpIdForLowLat = 0;
    uint32_t bwpIdForVoice = 0;
    uint32_t bwpIdForMmtc = 0;

    // gNb routing between Bearer and bandwidh part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("DGBR_DISCRETE_AUT_SMALL",
                                                 UintegerValue(bwpIdForLowLat));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForVoice));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT",
                                                 UintegerValue(bwpIdForMmtc));

    // Ue routing between Bearer and bandwidth part
    nrHelper->SetUeBwpManagerAlgorithmAttribute("DGBR_DISCRETE_AUT_SMALL",
                                                UintegerValue(bwpIdForLowLat));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB",
                                                UintegerValue(bwpIdForVoice));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_VIDEO_TCP_DEFAULT",
                                                UintegerValue(bwpIdForMmtc));

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */
    NetDeviceContainer gnbNetDev =
        nrHelper->InstallGnbDevice(gridScenario.GetBaseStations(), allBwps);
    NetDeviceContainer ueLowLatNetDev = nrHelper->InstallUeDevice(ueLowLatContainer, allBwps);
    NetDeviceContainer ueVoiceNetDev = nrHelper->InstallUeDevice(ueVoiceContainer, allBwps);
    NetDeviceContainer ueMmtcNetDev = nrHelper->InstallUeDevice(ueMmtcContainer, allBwps);

    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueLowLatNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueVoiceNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueMmtcNetDev, randomStream);

    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("Numerology", UintegerValue(numerology));
    nrHelper->GetGnbPhy(gnbNetDev.Get(0), 0)->SetAttribute("TxPower", DoubleValue(10 * log10(x)));

    // When all the configuration is done, explicitly call UpdateConfig ()
    for (auto it = gnbNetDev.Begin(); it != gnbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }

    for (auto it = ueLowLatNetDev.Begin(); it != ueLowLatNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = ueVoiceNetDev.Begin(); it != ueVoiceNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = ueMmtcNetDev.Begin(); it != ueMmtcNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // connect a remoteHost to pgw. Setup routing too
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.000)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(gridScenario.GetUserTerminals());

    Ipv4InterfaceContainer ueLowLatIpIface;
    Ipv4InterfaceContainer ueVoiceIpIface;
    Ipv4InterfaceContainer ueMmtcIpIface;
    ueLowLatIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLowLatNetDev));
    ueVoiceIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueVoiceNetDev));
    ueMmtcIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueMmtcNetDev));

    // Set the default gateway for the UEs
    for (uint32_t j = 0; j < gridScenario.GetUserTerminals().GetN(); ++j)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(
            gridScenario.GetUserTerminals().Get(j)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueLowLatNetDev, gnbNetDev);
    nrHelper->AttachToClosestGnb(ueVoiceNetDev, gnbNetDev);
    nrHelper->AttachToClosestGnb(ueMmtcNetDev, gnbNetDev);

    Ptr<BurstShieldShareScheduler> shareScheduler = DynamicCast<BurstShieldShareScheduler>(
        NrHelper::GetScheduler(gnbNetDev.Get(0), 0));
    NS_ABORT_MSG_IF(!shareScheduler, "Custom share scheduler was not installed");
    Simulator::Schedule(udpAppStartTime - MilliSeconds(1),
                        &InitializeShareScheduler,
                        shareScheduler,
                        ueLowLatNetDev,
                        ueVoiceNetDev,
                        ueMmtcNetDev);

    /*
     * Three downlink traffic proxies, each identified by destination port.
     */
    uint16_t dlPortLowLat = 1234;
    uint16_t dlPortVoice = 1235;
    uint16_t dlPortMmtc = 1236;

    ApplicationContainer serverApps;

    // The sink will always listen to the specified ports
    UdpServerHelper dlPacketSinkLowLat(dlPortLowLat);
    UdpServerHelper dlPacketSinkVoice(dlPortVoice);
    UdpServerHelper dlPacketSinkMmtc(dlPortMmtc);

    // The server, that is the application which is listening, is installed in the UE
    serverApps.Add(dlPacketSinkLowLat.Install(ueLowLatContainer));
    serverApps.Add(dlPacketSinkVoice.Install(ueVoiceContainer));
    serverApps.Add(dlPacketSinkMmtc.Install(ueMmtcContainer));

    /*
     * Configure attributes for the different generators, using user-provided
     * parameters for generating a CBR traffic
     *
     * Low-Latency configuration and object creation:
     */
    UdpClientHelper dlClientLowLat;
    dlClientLowLat.SetAttribute("RemotePort", UintegerValue(dlPortLowLat));
    dlClientLowLat.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientLowLat.SetAttribute("PacketSize", UintegerValue(lowLatPacketBytes));
    dlClientLowLat.SetAttribute("Interval", TimeValue(Seconds(1.0 / lowLatRatePps)));

    // Service-class proxies: these 5QIs do not themselves enforce the paper SLAs.
    NrEpsBearer lowLatBearer(NrEpsBearer::DGBR_DISCRETE_AUT_SMALL);

    // The filter for the low-latency traffic
    Ptr<NrEpcTft> lowLatTft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpfLowLat;
    dlpfLowLat.localPortStart = dlPortLowLat;
    dlpfLowLat.localPortEnd = dlPortLowLat;
    lowLatTft->Add(dlpfLowLat);

    // eMBB configuration and object creation:
    UdpClientHelper dlClientVoice;
    dlClientVoice.SetAttribute("RemotePort", UintegerValue(dlPortVoice));
    dlClientVoice.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientVoice.SetAttribute("PacketSize", UintegerValue(voicePacketBytes));
    dlClientVoice.SetAttribute("Interval", TimeValue(Seconds(1.0 / voiceRatePps)));

    NrEpsBearer voiceBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);

    // The filter for the voice traffic
    Ptr<NrEpcTft> voiceTft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpfVoice;
    dlpfVoice.localPortStart = dlPortVoice;
    dlpfVoice.localPortEnd = dlPortVoice;
    voiceTft->Add(dlpfVoice);

    UdpClientHelper dlClientMmtc;
    dlClientMmtc.SetAttribute("RemotePort", UintegerValue(dlPortMmtc));
    dlClientMmtc.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClientMmtc.SetAttribute("PacketSize", UintegerValue(mmtcPacketBytes));
    dlClientMmtc.SetAttribute("Interval", TimeValue(Seconds(1.0 / mmtcRatePps)));
    // No dedicated mMTC 5QI is modeled here; QCI 9 is a non-GBR proxy only.
    NrEpsBearer mmtcBearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT);
    Ptr<NrEpcTft> mmtcTft = Create<NrEpcTft>();
    NrEpcTft::PacketFilter dlpfMmtc;
    dlpfMmtc.localPortStart = dlPortMmtc;
    dlpfMmtc.localPortEnd = dlPortMmtc;
    mmtcTft->Add(dlpfMmtc);

    //  Install the applications
    ApplicationContainer clientApps;
    ApplicationContainer lowLatClients;
    ApplicationContainer voiceClients;
    ApplicationContainer mmtcClients;

    for (uint32_t i = 0; i < ueLowLatContainer.GetN(); ++i)
    {
        Ptr<NetDevice> ueDevice = ueLowLatNetDev.Get(i);
        Address ueAddress = ueLowLatIpIface.GetAddress(i);

        // The client, who is transmitting, is installed in the remote host,
        // with destination address set to the address of the UE
        dlClientLowLat.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        ApplicationContainer installed = dlClientLowLat.Install(remoteHost);
        clientApps.Add(installed);
        lowLatClients.Add(installed);

        // Activate a dedicated bearer for the traffic type
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, lowLatBearer, lowLatTft);
    }

    for (uint32_t i = 0; i < ueVoiceContainer.GetN(); ++i)
    {
        Ptr<NetDevice> ueDevice = ueVoiceNetDev.Get(i);
        Address ueAddress = ueVoiceIpIface.GetAddress(i);

        // The client, who is transmitting, is installed in the remote host,
        // with destination address set to the address of the UE
        dlClientVoice.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        ApplicationContainer installed = dlClientVoice.Install(remoteHost);
        clientApps.Add(installed);
        voiceClients.Add(installed);

        // Activate a dedicated bearer for the traffic type
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, voiceBearer, voiceTft);
    }

    for (uint32_t i = 0; i < ueMmtcContainer.GetN(); ++i)
    {
        Ptr<NetDevice> ueDevice = ueMmtcNetDev.Get(i);
        Address ueAddress = ueMmtcIpIface.GetAddress(i);
        dlClientMmtc.SetAttribute("RemoteAddress", AddressValue(ueAddress));
        ApplicationContainer installed = dlClientMmtc.Install(remoteHost);
        clientApps.Add(installed);
        mmtcClients.Add(installed);
        nrHelper->ActivateDedicatedEpsBearer(ueDevice, mmtcBearer, mmtcTft);
    }

    if (perSliceRlcCaps)
    {
        Simulator::Schedule(udpAppStartTime - MilliSeconds(1),
                            &ConfigurePerSliceRlc,
                            DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0)),
                            ueLowLatNetDev,
                            ueVoiceNetDev,
                            ueMmtcNetDev,
                            std::array<uint32_t, 3>{rlcCapUrllcBytes,
                                                    rlcCapEmbbBytes,
                                                    rlcCapMmtcBytes});
    }

    // start UDP server and client apps
    serverApps.Start(udpAppStartTime);
    clientApps.Start(udpAppStartTime);
    serverApps.Stop(simTime);
    clientApps.Stop(simTime);

    // The application interval is updated at the named stress boundaries. The
    // first packet after each boundary may still use the previous interval.
    if (!replaySchedule.empty())
    {
        NS_ABORT_MSG_IF(stressEvents != "nominal", "Replay requires disabled legacy stress");
        std::ifstream input(replaySchedule);
        NS_ABORT_MSG_IF(!input.is_open(), "Cannot open exogenous replay CSV");
        std::string line;
        std::getline(input, line);
        double previousTime = udpAppStartTime.GetSeconds();
        uint32_t records = 0;
        while (std::getline(input, line))
        {
            if (line.empty()) continue;
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream fields(line);
            double at, u, e, m, drop;
            NS_ABORT_MSG_IF(!(fields >> at >> u >> e >> m >> drop), "Malformed replay row");
            NS_ABORT_MSG_IF(!std::isfinite(at) || at <= previousTime || at >= simTime.GetSeconds() ||
                !std::isfinite(u) || !std::isfinite(e) || !std::isfinite(m) ||
                u < 1 || e < 1 || m < 1 || u > 100000 || e > 100000 || m > 100000 ||
                !std::isfinite(drop) || drop < 0 || drop > 40, "Invalid replay row");
            Simulator::Schedule(Seconds(at), &SetClientRate, lowLatClients, static_cast<uint32_t>(std::round(u)));
            Simulator::Schedule(Seconds(at), &SetClientRate, voiceClients, static_cast<uint32_t>(std::round(e)));
            Simulator::Schedule(Seconds(at), &SetClientRate, mmtcClients, static_cast<uint32_t>(std::round(m)));
            Simulator::Schedule(Seconds(at), &SetRadioPower, nrHelper, gnbNetDev, totalTxPower-drop);
            previousTime = at;
            ++records;
        }
        NS_ABORT_MSG_IF(records == 0, "Empty replay schedule");
    }
    else if (customStress)
    {
        const auto scaledRate = [](uint32_t rate, double scale) {
            const double value = std::round(rate * scale);
            NS_ABORT_MSG_IF(value < 1 || value > std::numeric_limits<uint32_t>::max(),
                            "Native event packet rate is invalid");
            return static_cast<uint32_t>(value);
        };
        for (const auto& event : nativeEvents)
        {
            Simulator::Schedule(Seconds(event.first), &SetClientRate, lowLatClients,
                                scaledRate(lowLatRatePps, trafficScaleUrllc));
            Simulator::Schedule(Seconds(event.first), &SetClientRate, voiceClients,
                                scaledRate(voiceRatePps, trafficScaleEmbb));
            Simulator::Schedule(Seconds(event.first), &SetClientRate, mmtcClients,
                                scaledRate(mmtcRatePps, trafficScaleMmtc));
            // Switch the cap just after the 100-ms policy boundary, so the
            // boundary's logged *previous* epoch retains its old cap.
            Simulator::Schedule(Seconds(event.first) + NanoSeconds(1),
                                &BurstShieldShareScheduler::SetCapacityScale,
                                shareScheduler, capacityScale);
            Simulator::Schedule(Seconds(event.second), &SetClientRate, lowLatClients,
                                lowLatRatePps);
            Simulator::Schedule(Seconds(event.second), &SetClientRate, voiceClients,
                                voiceRatePps);
            Simulator::Schedule(Seconds(event.second), &SetClientRate, mmtcClients,
                                mmtcRatePps);
            Simulator::Schedule(Seconds(event.second) + NanoSeconds(1),
                                &BurstShieldShareScheduler::SetCapacityScale,
                                shareScheduler, 1.0);
        }
    }
    else
    {
        Simulator::Schedule(Seconds(burstStartS), &SetClientRate, lowLatClients,
                            lowLatRatePps * burstFactor);
        Simulator::Schedule(Seconds(burstEndS), &SetClientRate, lowLatClients,
                            lowLatRatePps);
        // Legacy radio-impairment proxy; the new native matrix uses symbol caps.
        Simulator::Schedule(Seconds(impairmentStartS), &SetRadioPower, nrHelper,
                            gnbNetDev, totalTxPower - radioPowerDropDb);
        Simulator::Schedule(Seconds(impairmentEndS), &SetRadioPower, nrHelper,
                            gnbNetDev, totalTxPower);
    }

    // enable the traces provided by the nr module
    // nrHelper->EnableTraces();

    FlowMonitorHelper flowmonHelper;
    NodeContainer endpointNodes;
    endpointNodes.Add(remoteHost);
    endpointNodes.Add(gridScenario.GetUserTerminals());

    Ptr<ns3::FlowMonitor> monitor = flowmonHelper.Install(endpointNodes);
    monitor->SetAttribute("DelayBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("JitterBinWidth", DoubleValue(0.001));
    monitor->SetAttribute("PacketSizeBinWidth", DoubleValue(20));

    const Time kpiWindow = MilliSeconds(kpiWindowMs);
    Ptr<Ipv4FlowClassifier> windowClassifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    const std::string windowFilename = outputDir + "/" + simTag + "_windows.csv";
    const std::string packetFilename = outputDir + "/" + simTag + "_packets.csv";
    const std::string observationFilename = outputDir + "/" + simTag + "_control_observations.csv";
    const std::string actionFilename = outputDir + "/" + simTag + "_share_actions.csv";
    const std::string cohortFilename = outputDir + "/" + simTag + "_deadline_cohorts.csv";
    const std::string deadlinePacketFilename = outputDir + "/" + simTag + "_deadline_packets.csv";
    const std::string pdTransferFilename = outputDir + "/" + simTag +
        (shareMode == "external" ? "_external_bridge_actions.csv" : "_pd_transfer_actions.csv");
    WindowRecorder recorder(monitor,
                            windowClassifier,
                            windowFilename,
                            packetFilename,
                            observationFilename,
                            cohortFilename,
                            deadlinePacketFilename,
                            kpiWindow,
                            simTime,
                            udpAppStartTime,
                            dlPortLowLat,
                            dlPortVoice,
                            dlPortMmtc,
                            lowLatSlaMs,
                            voiceSlaMs,
                            mmtcSlaMs,
                            burstStartS,
                            burstEndS,
                            impairmentStartS,
                            impairmentEndS,
                            nativeEvents,
                            customStress,
                            nativeTraffic,
                            nativeCapacity,
                            changedSlaProfile);
    V4Telemetry v4(outputDir + "/" + simTag);
    Simulator::Schedule(udpAppStartTime - NanoSeconds(1), &V4Telemetry::Attach,
        &v4, DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0)), ueLowLatNetDev, ueVoiceNetDev, ueMmtcNetDev);
    serverApps.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&V4Telemetry::RxU, &v4));
    serverApps.Get(1)->TraceConnectWithoutContext("Rx", MakeCallback(&V4Telemetry::RxE, &v4));
    serverApps.Get(2)->TraceConnectWithoutContext("Rx", MakeCallback(&V4Telemetry::RxM, &v4));
    NS_ABORT_MSG_IF(!remoteHost->GetObject<Ipv4L3Protocol>()->TraceConnectWithoutContext(
                        "SendOutgoing", MakeCallback(&WindowRecorder::OnIpTx, &recorder)),
                    "Unable to attach post-header IPv4 send trace");
    NS_ABORT_MSG_IF(!serverApps.Get(0)->TraceConnectWithoutContext(
                        "Rx", MakeCallback(&WindowRecorder::OnLowLatRx, &recorder)) ||
                        !serverApps.Get(1)->TraceConnectWithoutContext(
                            "Rx", MakeCallback(&WindowRecorder::OnVoiceRx, &recorder)) ||
                        !serverApps.Get(2)->TraceConnectWithoutContext(
                            "Rx", MakeCallback(&WindowRecorder::OnMmtcRx, &recorder)),
                    "Unable to attach UdpServer packet traces");
    if (telemetryV2)
    {
        Simulator::Schedule(udpAppStartTime - MilliSeconds(1),
                            &AttachRlcOverflowTraces,
                            DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(0)),
                            ueLowLatNetDev,
                            ueVoiceNetDev,
                            ueMmtcNetDev,
                            &recorder);
    }
    Simulator::Schedule(udpAppStartTime + kpiWindow, &WindowRecorder::Sample, &recorder);
    Simulator::Schedule(udpAppStartTime + Seconds(1) + NanoSeconds(1),
                        &WindowRecorder::SampleCohort,
                        &recorder);
    std::unique_ptr<FilePolicyBridge> pdBridge;
    if (shareMode == "pd" || shareMode == "external")
    {
        pdBridge = std::make_unique<FilePolicyBridge>(bridgeDir, pdTransferFilename);
    }
    ShareController shareController(shareScheduler,
                                    actionFilename,
                                    shareMode,
                                    kpiWindow,
                                    simTime,
                                    burstStartS,
                                    std::max(burstEndS, impairmentEndS),
                                    [&recorder, &pdBridge, &v4, shareScheduler, telemetryV2,
                                     shareMode](Time decisionTime) {
                                        v4.Snapshot(decisionTime);
                                        if (pdBridge)
                                        {
                                            auto observation = recorder.PolicyObservation(
                                                decisionTime, shareScheduler, telemetryV2);
                                            if (shareMode == "external")
                                            {
                                                // Causal extras for paper-policy ports. Existing 61-field
                                                // frozen-PD request remains unchanged.
                                                const auto receivedBits = recorder.GetLastWindowRxBits();
                                                const auto dlMcs = shareScheduler->GetLastMcs();
                                                observation.insert(observation.end(),
                                                                   receivedBits.begin(),
                                                                   receivedBits.end());
                                                observation.insert(observation.end(),
                                                                   dlMcs.begin(),
                                                                   dlMcs.end());
                                            }
                                            const auto result = pdBridge->Decide(
                                                decisionTime, observation);
                                            recorder.SetPreviousPolicyAction(result.first);
                                            return result.second;
                                        }
                                        return recorder.ReactiveShares(decisionTime);
                                    });
    Simulator::Schedule(udpAppStartTime + kpiWindow, &ShareController::Tick, &shareController);

    // Allow the final KPI snapshot at simTime to run before stopping.
    Simulator::Stop(simTime + NanoSeconds(1));
    Simulator::Run();
    std::cout << "Window KPI CSV: " << windowFilename << std::endl;
    std::cout << "Packet delay CSV: " << packetFilename << std::endl;
    std::cout << "Causal control-observation CSV: " << observationFilename << std::endl;
    std::cout << "Requested and assigned symbol CSV: " << actionFilename << std::endl;
    std::cout << "One-second deadline-cohort CSV: " << cohortFilename << std::endl;
    if (pdBridge)
    {
        std::cout << "Policy bridge raw-vs-quota actions CSV: " << pdTransferFilename
                  << std::endl;
    }

    /*
     * To check what was installed in the memory, i.e., BWPs of gNB Device, and its configuration.
     * Example is: Node 1 -> Device 0 -> BandwidthPartMap -> {0,1} BWPs -> NrGnbPhy -> Numerology,
    GtkConfigStore config;
    config.ConfigureAttributes ();
    */

    // Print per-flow statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    double averageFlowThroughput = 0.0;
    double averageFlowDelay = 0.0;

    std::ofstream outFile;
    std::string filename = outputDir + "/" + simTag;
    outFile.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile.is_open())
    {
        std::cerr << "Can't open file " << filename << std::endl;
        return 1;
    }

    outFile.setf(std::ios_base::fixed);

    double flowDuration = (simTime - udpAppStartTime).GetSeconds();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::stringstream protoStream;
        protoStream << (uint16_t)t.protocol;
        if (t.protocol == 6)
        {
            protoStream.str("TCP");
        }
        if (t.protocol == 17)
        {
            protoStream.str("UDP");
        }
        outFile << "Flow " << i->first << " (" << t.sourceAddress << ":" << t.sourcePort << " -> "
                << t.destinationAddress << ":" << t.destinationPort << ") proto "
                << protoStream.str() << "\n";
        outFile << "  Tx Packets: " << i->second.txPackets << "\n";
        outFile << "  Tx Bytes:   " << i->second.txBytes << "\n";
        outFile << "  TxOffered:  " << i->second.txBytes * 8.0 / flowDuration / 1000.0 / 1000.0
                << " Mbps\n";
        outFile << "  Rx Bytes:   " << i->second.rxBytes << "\n";
        if (i->second.rxPackets > 0)
        {
            // Measure the duration of the flow from receiver's perspective
            averageFlowThroughput += i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000;
            averageFlowDelay += 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets;

            outFile << "  Throughput: " << i->second.rxBytes * 8.0 / flowDuration / 1000 / 1000
                    << " Mbps\n";
            outFile << "  Mean delay:  "
                    << 1000 * i->second.delaySum.GetSeconds() / i->second.rxPackets << " ms\n";
            // outFile << "  Mean upt:  " << i->second.uptSum / i->second.rxPackets / 1000/1000 << "
            // Mbps \n";
            outFile << "  Mean jitter:  "
                    << 1000 * i->second.jitterSum.GetSeconds() / i->second.rxPackets << " ms\n";
        }
        else
        {
            outFile << "  Throughput:  0 Mbps\n";
            outFile << "  Mean delay:  0 ms\n";
            outFile << "  Mean jitter: 0 ms\n";
        }
        outFile << "  Rx Packets: " << i->second.rxPackets << "\n";
    }

    outFile << "\n\n  Mean flow throughput: " << averageFlowThroughput / stats.size() << "\n";
    outFile << "  Mean flow delay: " << averageFlowDelay / stats.size() << "\n";

    outFile.close();

    std::ifstream f(filename.c_str());

    if (f.is_open())
    {
        std::cout << f.rdbuf();
    }

    Simulator::Destroy();
    return 0;
}

int main(int argc,char* argv[]) { if(argc==2 && std::string(argv[1])=="--v4-self-test") return ns3::V4SelfTest(); return FrozenMain(argc,argv); }
