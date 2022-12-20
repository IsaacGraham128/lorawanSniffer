# Used to initialise the necessary components of the sniffer

# Reset the cellular card
#./reset_cell.sh
#echo "Cellular card reset"

# Start the openVPN client daemon
#sudo openvpn --config /etc/openvpn/sniffer.ovpn &
#echo "OpenVPN client started"

# Reset the lgw
./reset_lgw.sh start
echo "LoRaWAN concentrator reset and ready to go"

# Start the script
./sniffer -d > always4.log
echo "Sniffer started"
