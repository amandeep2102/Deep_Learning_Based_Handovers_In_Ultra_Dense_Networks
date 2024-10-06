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

NS_LOG_COMPONENT_DEFINE("ScratchSimulator");

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
    uint16_t numberOfUes = 1;
    uint16_t numberOfEnbs = 7;
    Time simTime = Seconds(100);
    // double distance = 50;
    Time interval = Seconds(50);
    double speed = 10;
    bool verbose = false;
    std::string dbFileName = "oran-repository.db";

    // Command line arguments
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.Parse(argc, argv);

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
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm"); // disable automatic
    // handover
    // lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // Install Mobility Model
    // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    // for (uint16_t i = 0; i < numberOfEnbs; i++)
    // {
    //     positionAlloc->Add(Vector(distance * i, 0, 20));
    // }

    // for (uint16_t i = 0; i < numberOfUes; i++)
    // {
    //     // Coordinates of the middle point between the eNBs, minus the distance covered
    //     // in half of the interval for switching directions
    //     positionAlloc->Add(Vector((distance / 2) - (speed * (interval.GetSeconds() / 2)),
    //     0, 1.5));
    // }

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(200, 616, 0));
    positionAlloc->Add(Vector(400, 616, 0));
    positionAlloc->Add(Vector(600, 616, 0));
    positionAlloc->Add(Vector(800, 616, 0));
    positionAlloc->Add(Vector(266, 516, 0));
    positionAlloc->Add(Vector(466, 516, 0));
    positionAlloc->Add(Vector(666, 516, 0));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);

    Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
    uePositionAlloc->Add(Vector(90, 570, 0));

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.SetPositionAllocator(uePositionAlloc);
    mobility.Install(ueNodes);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(speed, 0, 0));
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

    // Attach all UEs to the first eNodeB
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
    }

    // Add X2 interface
    lteHelper->AddX2Interface(enbNodes);

    PrintGnuplottableEnbListToFile("scratch/enblist");

    PrintGnuplottableUeListToFile("scratch/uelist");

    // ORAN Models -- BEGIN
    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }
    Ptr<OranDataRepository> dataRepository = CreateObject<OranDataRepositorySqlite>();
    Ptr<OranLm> defaultLm = CreateObject<OranLmLte2LteDistanceHandover>();
    Ptr<OranCmm> cmm = CreateObject<OranCmmNoop>();
    Ptr<OranNearRtRic> nearRtRic = CreateObject<OranNearRtRic>();
    Ptr<OranNearRtRicE2Terminator> nearRtRicE2Terminator =
        CreateObject<OranNearRtRicE2Terminator>();

    dataRepository->SetAttribute("DatabaseFile", StringValue(dbFileName));
    if (verbose)
    {
        dataRepository->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
    }

    defaultLm->SetAttribute("Verbose", BooleanValue(true));
    defaultLm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
    defaultLm->SetAttribute("ProcessingDelayRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    cmm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
    cmm->SetAttribute("Verbose", BooleanValue(true));

    nearRtRicE2Terminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
    nearRtRicE2Terminator->SetAttribute("DataRepository", PointerValue(dataRepository));
    nearRtRicE2Terminator->SetAttribute("TransmissionDelayRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

    nearRtRic->SetAttribute("DefaultLogicModule", PointerValue(defaultLm));
    nearRtRic->SetAttribute("E2Terminator", PointerValue(nearRtRicE2Terminator));
    nearRtRic->SetAttribute("DataRepository", PointerValue(dataRepository));
    nearRtRic->SetAttribute("LmQueryInterval", TimeValue(Seconds(5)));
    nearRtRic->SetAttribute("ConflictMitigationModule", PointerValue(cmm));
    nearRtRic->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    nearRtRic->SetAttribute("E2NodeInactivityIntervalRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    nearRtRic->SetAttribute("LmQueryMaxWaitTime",
                            TimeValue(Seconds(0))); // 0 means wait for all LMs to finish
    nearRtRic->SetAttribute("LmQueryLateCommandPolicy", EnumValue(OranNearRtRic::DROP));

    Simulator::Schedule(Seconds(1), &OranNearRtRic::Start, nearRtRic);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> lteUeCellInfoReporter =
            CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranE2NodeTerminatorLteUe> lteUeTerminator = CreateObject<OranE2NodeTerminatorLteUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
        locationReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        lteUeCellInfoReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
        lteUeCellInfoReporter->SetAttribute(
            "Trigger",
            StringValue("ns3::OranReportTriggerLteUeHandover[InitialReport=true]"));

        lteUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteUeTerminator->SetAttribute("RegistrationIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteUeTerminator->SetAttribute("SendIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteUeTerminator->SetAttribute("TransmissionDelayRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

        lteUeTerminator->AddReporter(locationReporter);
        lteUeTerminator->AddReporter(lteUeCellInfoReporter);

        lteUeTerminator->Attach(ueNodes.Get(idx));

        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorLteUe::Activate, lteUeTerminator);
    }

    for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranE2NodeTerminatorLteEnb> lteEnbTerminator =
            CreateObject<OranE2NodeTerminatorLteEnb>();

        locationReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));
        locationReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        lteEnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteEnbTerminator->SetAttribute("RegistrationIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteEnbTerminator->SetAttribute("SendIntervalRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteEnbTerminator->SetAttribute("TransmissionDelayRv",
                                       StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

        lteEnbTerminator->AddReporter(locationReporter);

        lteEnbTerminator->Attach(enbNodes.Get(idx));

        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorLteEnb::Activate, lteEnbTerminator);
    }

    // ORAN Models -- END

    // Trace the end of handovers
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));

    Simulator::Stop(simTime);
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
