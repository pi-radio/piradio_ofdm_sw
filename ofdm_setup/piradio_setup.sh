insmod driver/piradio0.ko &&
ip link set piradio00 mtu 8000 &&
ip link set piradio01 mtu 8000 &&
ip link set piradio00 up &&
ip link set piradio01 up &&
./zcu111_setup.elf -r 2 -t sync_temp.bin -m map.bin -p &&
ip addr add 192.168.4.4/24 dev piradio00 &&
ip addr add 192.168.5.5/24 dev piradio01 &&
devmem2 0xB0010000 h 3 &&
devmem2 0xB0011000 h 3
