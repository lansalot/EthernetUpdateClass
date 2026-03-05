// Usual AOG firmware sections here, nothing special
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
struct ConfigIP
{
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 1;
};
ConfigIP networkAddress; // 3 bytes
byte Eth_myip[4] = {0, 0, 0, 0}; // This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};
IPAddress Eth_ipDestination;
// End of usual AOG firmware sections


// the Ethernetupdater, these two lines are all you need in declarations
#include "EthernetUpdater.h"
EthernetUpdater updater;
// End of Ethernetupdater declarations

// Setup procedure ------------------------
void setup()
{
    delay(1000);
    Serial.begin(115200);
    delay(100);
    Serial.println("Start setup");
    EthernetStart();
	// Initialise the updater after the Ethernet is up and running, otherwise it will fail to start
    updater.begin();
    Serial.println("\r\nEnd setup, loop time!");
}

void loop()
{
    // poll once in your usual AOG loop(), that's it! No more changes
    updater.poll();
}