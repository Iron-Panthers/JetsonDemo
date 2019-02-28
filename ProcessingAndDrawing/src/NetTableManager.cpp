#include "NetTableManager.hpp"
#include "helper.hpp"

static string tableName = "CVResultsTable"; // name of the table
string netTableAddress = "192.168.1.34"; //address of the rio

void NetTableManager::init()
{
    // initialize NetworkTables
    NetworkTable::SetClientMode();
    NetworkTable::SetDSClientEnabled(false);
    NetworkTable::SetIPAddress(llvm::StringRef(netTableAddress));
    NetworkTable::Initialize();
    printf("Initialized table\n");
    networkTable = NetworkTable::GetTable(tableName);
}

static void NetTableManager::pushToNetworkTables(VisionResultsPackage info)
{
    if (instance == NULL) {
        instance = NetTableManager();
    }
    
    networkTable->PutString("VisionResults", info.createCSVLine());
    networkTable->PutString("VisionResultsHeader", info.createCSVHeader());
    networkTable->PutNumber("Sample Hue", info.sampleHue);
    networkTable->PutNumber("Sample Sat", info.sampleSat);
    networkTable->PutNumber("Sample Val", info.sampleVal);
    networkTable->Flush();
}