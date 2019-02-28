#include "NetTableManager.hpp"
#include "helper.hpp"

static string tableName = "CVResultsTable"; // name of the table
string netTableAddress = "192.168.1.34"; //address of the rio
NetTableManager* NetTableManager::instance = 0;

NetTableManager::NetTableManager()
{
    // initialize NetworkTables
    NetworkTable::SetClientMode();
    NetworkTable::SetDSClientEnabled(false);
    NetworkTable::SetIPAddress(llvm::StringRef(netTableAddress));
    NetworkTable::Initialize();
    printf("Initialized table\n");
    networkTable = NetworkTable::GetTable(tableName);
}

NetTableManager* NetTableManager::getInstance()
{
    if (instance == NULL)
    {
        instance = new NetTableManager();
    }
    return instance;
}

void NetTableManager::pushToNetworkTables(VisionResultsPackage info)
{
    this->networkTable->PutString("VisionResults", info.createCSVLine());
    this->networkTable->PutString("VisionResultsHeader", info.createCSVHeader());
    this->networkTable->Flush();
}