insmod ../../driver/piradio0.ko project_type=0 &&
ip link set piradio00 mtu 8000 &&
ip link set piradio00 up &&
./zcu111_setup.elf -r 2 -t sync_temp.bin -m map.bin -p &&
ip addr add 192.168.4.4/24 dev piradio00 &&
devmem2 0xB0010000 h 3
