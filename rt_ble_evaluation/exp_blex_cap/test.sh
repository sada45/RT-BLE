#! /bin/bash
BOARD=nrf52840dk make all -j8
python3 /home/user/JLink/mass_flash.py -s 1 -f ./bin/nrf52840dk/periodic_server.bin -e
python3 /home/user/JLink/rtt_view.py -vt -s 1
