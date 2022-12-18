#! /bin/bash
python3 /home/user/JLink/mass_flash.py -s 1 -f ./bin/nrf52840dk/periodic_client.bin
python3 /home/user/JLink/rtt_view.py -vt -s 1
