#include "NetTableManager.hpp"
#include "helper.hpp"

static string tableName = "CVResultsTable"; // name of the table
string netTableAddress = "10.50.26.2"; //address of the rio
NetTableManager* NetTableManager::instance = 0;

NetTableManager::NetTableManager()
{
    // initialize NetworkTables
    NetworkTable::SetClientMode();
    NetworkTable::SetDSClientEnabled(false);
    NetworkTable::SetIPAddress(llvm::StringRef(netTableAddress));
    NetworkTable::SetPort(1735);
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