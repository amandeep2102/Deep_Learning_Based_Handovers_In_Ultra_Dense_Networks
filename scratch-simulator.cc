/*
        -------------------------------------------------------------------------
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -                                                                       -
        -------------------------------------------------------------------------


*/

#include <ns3/core-module.h>
#include <ns3/internet-module.h>
#include <ns3/lte-module.h>
#include <ns3/mobility-module.h>
#include <ns3/network-module.h>
#include <ns3/oran-module.h>

#include <stdio.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranLte2LteDeepLearningDistanceHandover");

void
PrintGnuplottableUeListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename, std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<LteUeNetDevice> uedev = node->GetDevice(j)->GetObject<LteUeNetDevice>();
            if (uedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                // outFile << "set label \"" << uedev->GetImsi() << "\" at " << pos.x << "," <<
                // pos.y
                //         << " left font \"Helvetica,4\" textcolor rgb \"grey\" front point pt 1 ps
                //         "
                //            "0.3 lc rgb \"grey\" offset 0,0"
                //         << std::endl;
                outFile << pos.x << " " << pos.y << std::endl;
            }
        }
    }
}

void
PrintGnuplottableEnbListToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename, std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<LteEnbNetDevice> enbdev = node->GetDevice(j)->GetObject<LteEnbNetDevice>();
            if (enbdev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                // outFile << "set label \"" << enbdev->GetCellId() << "\" at " << pos.x << ","
                //         << pos.y
                //         << " left font \"Helvetica,4\" textcolor rgb \"white\" front  point pt 2"
                //         << " ps 0.3 lc rgb \"white\" offset 0,0" << std::endl;
                outFile << "enb" << enbdev->GetCellId() << " " << pos.x << " " << pos.y
                        << std::endl;
            }
        }
    }
}

void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " eNB CellId " << cellid
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}

void
ReverseVelocity(NodeContainer nodes, Time interval)
{
    for (uint32_t idx = 0; idx < nodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            nodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(mobility->GetVelocity().x * -1, 0, 0));
    }

    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}

void
QueryRcSink(std::string query, std::string args, int rc)
{
    std::cout << Simulator::Now().GetSeconds() << " Query "
              << ((rc == SQLITE_OK || rc == SQLITE_DONE) ? "OK" : "ERROR") << "(" << rc << "): \""
              << query << "\"";

    if (!args.empty())
    {
        std::cout << " (" << args << ")";
    }
    std::cout << std::endl;
}

/**
 * ORAN handover example. Based on the LTE module's "lena-x2-handover.cc".
 */
int
main(int argc, char* argv[])
{
    uint16_t numberOfUes = 5;
    uint16_t numberOfEnbs = 7;
    Time simTime = Seconds(100);
    // double distance = 50;
    Time interval = Seconds(50);
    double speed[5]={100,50,80,120,20};
    bool verbose = false;
    std::string dbFileName = "oran-repository.db";

    // Command line arguments
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.Parse(argc, argv);

    // LogComponentEnable("LteHelper", LOG_LEVEL_INFO);
    // LogComponentEnable("LteEnbRrc", LOG_LEVEL_INFO);
    LogComponentEnable("LteUeRrc", LOG_LEVEL_INFO); 


    // Models the transmission of RRC messages from the UE to the eNB in a real fashion, by creating real RRC PDUs and
    // transmitting them over Signaling Radio Bearers using radio resources allocated by the LTE MAC scheduler
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));
    // Disabled to prevent the automatic cell reselection when signal quality is bad.
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(true));
    // transmission power
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(100));

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisPropagationLossModel"));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(50));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(50));
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    // disable automatic handover
    // lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm"); 
    // Implementation of the strongest cell handover algorithm, based on RSRP measurements and Event A3
    lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // Install Mobility Model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(2000, 6160, 0));
    positionAlloc->Add(Vector(4000, 6160, 0));
    positionAlloc->Add(Vector(6000, 6160, 0));
    positionAlloc->Add(Vector(8000, 6160, 0));
    positionAlloc->Add(Vector(2662, 5160, 0));
    positionAlloc->Add(Vector(4662, 5160, 0));
    positionAlloc->Add(Vector(6702, 5160, 0));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);

    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    uePositionAlloc->Add(Vector(900, 5500, 0));
    uePositionAlloc->Add(Vector(2200,4500,0));
    uePositionAlloc->Add(Vector(3000,6000,0));
    uePositionAlloc->Add(Vector(4400,5300,0));
    uePositionAlloc->Add(Vector(6000,6500,0));

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(speed[idx], 0, 0));
    }

    // Schedule the first direction switch
    // Simulator::Schedule(interval, &ReverseVelocity, ueNodes, interval);

    // Install LTE Devices in eNB and UEs
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);
    // Install the IP stack on the UEs
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    // Attach all UEs to the closest eNodeB
    lteHelper->Attach(ueLteDevs);

    // Add X2 interface
    lteHelper->AddX2Interface(enbNodes);

    PrintGnuplottableEnbListToFile("scratch/enblist");

    PrintGnuplottableUeListToFile("scratch/uelist");

    // ORAN Models -- BEGIN
    Ptr<OranNearRtRic> nearRtRic = nullptr;
    OranE2NodeTerminatorContainer e2NodeTerminatorsEnbs;
    OranE2NodeTerminatorContainer e2NodeTerminatorsUes;
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();

    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(Seconds(5)));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime",
                             TimeValue(Seconds(0))); // 0 means wait for all LMs to finish
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", EnumValue(OranNearRtRic::DROP));
    oranHelper->SetAttribute("RicTransmissionDelayRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

    // RIC setup
    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }

    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteDistanceHandover",
                                      "ProcessingDelayRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    nearRtRic = oranHelper->CreateNearRtRic();

    // UE Nodes setup
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorLteUe",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "TransmissionDelayRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));

    oranHelper->AddReporter("ns3::OranReporterLteUeCellInfo",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerLteUeHandover[InitialReport=true]"));

    e2NodeTerminatorsUes.Add(oranHelper->DeployTerminators(nearRtRic, ueNodes));

    // ENb Nodes setup
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorLteEnb",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "TransmissionDelayRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));

    e2NodeTerminatorsEnbs.Add(oranHelper->DeployTerminators(nearRtRic, enbNodes));

    // DB logging to the terminal
    if (verbose)
    {
        nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
    }

    // Activate and the components
    Simulator::Schedule(Seconds(1), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsEnbs);
    Simulator::Schedule(Seconds(2),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsUes);      
    // ORAN Models -- END

    // Trace the end of handovers
    // Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
    //                 MakeCallback(&NotifyHandoverEndOkEnb));

    Simulator::Stop(simTime);
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
