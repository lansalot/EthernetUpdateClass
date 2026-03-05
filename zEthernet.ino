// Typically this is already in your existing firmware, don't be copying this. Nothing in here relevant to the updater part!!

void EthernetStart()
{
    Serial.println("Initializing ethernet with static IP address");

    // try to congifure using IP:
    Ethernet.begin(mac, 0);          // Start Ethernet with IP 0.0.0.0

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
    Eth_myip[3] = 226;  //226 is tool steer

    Ethernet.setLocalIP(Eth_myip);
    Serial.println("\r\nEthernet status OK");
    Serial.print("IP set Manually: ");
    Serial.println(Ethernet.localIP());

    Eth_ipDestination[0] = Eth_myip[0];
    Eth_ipDestination[1] = Eth_myip[1];
    Eth_ipDestination[2] = Eth_myip[2];
    Eth_ipDestination[3] = 255;
}
