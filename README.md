# RT-BLE
This is the open source code for the paper accepted by IEEE INFOCOM 2023: [RT-BLE: Real-time Multi-Connection Scheduling for Bluetooth Low Energy](http://)

This work is based on the [RIOT OS](https://github.com/RIOT-OS/RIOT) and [Apache NimBLE](https://github.com/apache/mynewt-nimble). This repository include the RIOT OS and the NimBLE is a third-party package of the RIOT. Our modified NimBLE can be find [here](https://github.com/sada45/RT-BLE-NimBLE). Each modification should have a comment starting with `"RT-BLE:"`. 

The main part of the _RT-BLE_ is implemented as a system module name `rt_ble`. The code is located in `sys/rt_ble`, header file is located in `sys/include/rt_ble.h`

We use the Nordic nRF52840DK compatible boards. SEGGER RTT is recommended to be used as the standard output method. The makefile has already modified to use the RTT. We have a simple [CMD line tool](https://github.com/sada45/rtt_cmd_tool) for RTT log reading and mass flash. The code may be ugly, but really useful. Hope it is helpful for you too :-)

## Get started
1. First, you should install the required tool chains. Normally on Ubuntu, these two commands can install all the requirements.
```shell
sudo apt install build-essential
sudo apt install gcc-arm-none-eabi
```
2. Then pull this repository (The NimBLE can be automatically downloaded when you first time make any application). After this, enter the `rt_ble_evaluation` directory.
3. Each test has two folders, the one ended with `_client` should be burned to the Central node (Master), and the one ended with `_server` should be burned to the Peripheral node (Slave). For the first time, you should change the MAC address in `main.c` to the MAC address of your boards.
4. The following command can be used to make the application and then you can burn the binary file in `./bin/nrf52840dk/exp_<NAME>_<ROLE>.bin` to your boards with official JLink tools or the cmd line tool mentioned above.
```shell
make TARGET=nrf52840dk all -j16
```

## The tests included

|Folder Name|Description|
|-----------|-----------|
|exp\_rtble\_\*, exp\_blex\_\*, exp\_nf\_\*|This is the basic test to measure the delay of RT-BLE, BLEX and TL-BLE.
|exp\_rtble\_critical\_\*, exp\_blex\_critical\_\*|This is the test to get the worst-case latency during requirement changes (enter or leave the critical mode)|
|exp\_rtble_moveanchor\_\*, exp\_blex\_moveanchor\_\*|This is the test to get the connection reschedule time for RT-BLE and BLEX|
|exp\_rtble\_cap\_\*, exp\_blex\_cap\_\*|This is the test to get the system capacity of the RT-BLE and BLEX|

For the TL-BLE, we should first measure the average retransmission time $n_f$. The result measured in our experiment setup is:

<!-- packet_loss = [0, 10, 20, 30, 40]
nf = [1.0, 1.1395, 1.3239, 1.5297, 1.7577] -->
|Packet Loss Rate (%)|$n_f$|
|--------------------|--------------------|
|0|1.0|
|10|1.1395|
|20|1.3239|
|30|1.5297|
|40|1.7577|

