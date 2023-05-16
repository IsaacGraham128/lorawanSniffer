# Used to initialise the necessary components of the sniffer

# Reset the cellular card
#./reset_cell.sh
#echo "Cellular card reset"

# Start the openVPN client daemon
#sudo openvpn --config /etc/openvpn/sniffer.ovpn &
echo "Reseting OpenVPN client"
sudo ifconfig tun0 down
sudo ifconfig tun0 up



# Start the script
./sniffer -d > always4.log
echo "Sniffer started"
