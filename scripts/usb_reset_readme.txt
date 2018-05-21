1. Compile the program:

$ cc usb_reset.c -o usb_reset

2. Get the Bus and Device ID of the USB device you want to reset:

$ lsusb  
Bus 002 Device 003: ID 0fe9:9010 DVICO  

3. Make our compiled program executable:

$ chmod +x usb_reset

4. Execute the program with sudo privilege; make necessary substitution for <Bus> and <Device> ids as found by running the lsusb command:

$ sudo ./usb_reset /dev/bus/usb/002/003  
