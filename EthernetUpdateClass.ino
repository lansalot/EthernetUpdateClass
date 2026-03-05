//#include <NativeEthernet.h>
//#include <NativeEthernetUdp.h>

struct ConfigIP
{
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 1;
};
ConfigIP networkAddress; // 3 bytes

// IP & MAC address of this module of this module
byte Eth_myip[4] = {0, 0, 0, 0}; // This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

IPAddress Eth_ipDestination;

#include "EthernetUpdater.h"

EthernetUpdater updater();


// Setup procedure ------------------------
void setup()
{
    delay(1000);
    Serial.begin(115200);
    delay(100);
    Serial.println("Start setup");
    EthernetStart();
    updater.begin();
    Serial.println("\r\nEnd setup, loop time!");
}

void loop()
{
    updater.poll();
}