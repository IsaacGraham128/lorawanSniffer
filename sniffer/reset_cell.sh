echo 'AT+QENG="servingcell"\r' > /dev/ttyUSB2

sleep 0.5s

echo 'AT+QCFG="usbnet",1\r' > /dev/ttyUSB2

sleep 0.5s

echo 'AT+CFUN=1,1\r' > /dev/ttyUSB2

sleep 0.5s