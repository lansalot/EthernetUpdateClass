# EthernetUpdater (Teensy + NativeEthernet)

Minimal firmware update integration for AOG-style projects.

Use the updater found here: https://github.com/SK21/EthernetUpdate/tree/main/UpdateDemoApp

(Thanks David!)

And in time I'll integrate it into AogConfigOMatic

## What to copy into your project

- `EthernetUpdater.h`
- `EthernetUpdater.cpp`
- `FlashTxx.h`
- `FlashTxx.c`

## Prerequisites

- Project already uses `NativeEthernet` / `NativeEthernetUdp`.
- Your normal Ethernet init (`Ethernet.begin(...)`, `Ethernet.setLocalIP(...)`) is already working.

## Integration (minimal)

In your declarations:

```cpp
#include "EthernetUpdater.h"
EthernetUpdater updater;
```

In `setup()` (after Ethernet is started):

```cpp
EthernetStart();
updater.begin();
```

in `loop()`:

```cpp
  updater.sendHeartbeat();
```

At the end of your AOG `udpReceive`:`:

```cpp
  {
    // Final check: updater parses this packet from the main UDP path without consuming another UDP packet.
    updater.checkPacket(udpData, sizeToRead, src_ip);
  }
```

## Built-in behavior

- Receive port is fixed: `29100`
- Send port is fixed: `29000`
- Destination IP is auto-derived as local `/24` broadcast (`x.y.z.255`)

## Notes

- Call `updater.begin()` only after Ethernet is up, or the updater socket will not start.
- call `updater.sendHeartbeat();` in your main loop
- Don't forget to add `checkPacket` to the end of the main AOG UDP receive function
