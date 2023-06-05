#!/bin/bash
# Used to initialise the necessary components of the sniffer

# Reset the cellular card
#./reset_cell.sh
#echo "Cellular card reset"

# Start the openVPN client daemon
#sudo openvpn --config /etc/openvpn/sniffer.ovpn &
#echo "OpenVPN client started"

# Start the script
./stinker_server -d 1> stinker_server_debug.txt 2> stinker_server_debug.txt  &
sleep 4
./stinker_client -d