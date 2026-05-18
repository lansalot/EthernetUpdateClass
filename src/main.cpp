#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include "EthernetUpdater.h"

struct ConfigIP
{
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 1;
};

ConfigIP networkAddress; // 3 bytes
byte Eth_myip[4] = {0, 0, 0, 0}; // Set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

EthernetUpdater updater;

void EthernetStart()
{
    Serial.println("Initializing ethernet with static IP address");

    Ethernet.begin(mac, 0); // Start Ethernet with IP 0.0.0.0

    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
        Serial.println("Ethernet shield was not found. GPS via USB only.");
        return;
    }

    if (Ethernet.linkStatus() == LinkOFF)
    {
        Serial.println("Ethernet cable is not connected - Who cares we will start ethernet anyway.");
    }

    Eth_myip[0] = networkAddress.ipOne;
    Eth_myip[1] = networkAddress.ipTwo;
    Eth_myip[2] = networkAddress.ipThree;
    Eth_myip[3] = 126;

    Ethernet.setLocalIP(Eth_myip);
    Serial.println("\r\nEthernet status OK");
    Serial.print("IP set Manually: ");
    Serial.println(Ethernet.localIP());
}

void setup()
{
    delay(1000);
    Serial.begin(115200);
    delay(100);
    Serial.println("Start setup");

    EthernetStart();
    // Initialize updater after Ethernet is up, otherwise socket startup fails.
    updater.begin();

    Serial.println("\r\nEnd setup, loop time!");
}

void loop()
{
    updater.poll();
}
