Linux driver support FPGA with NVRAM, UART, I2C.
Iterface of FPGA is PCI Express bus.


Build driver:

	1. Need sources Linux in /usr/src/linux.
	2. Run from current dir		sudo ./mod_make.sh


Load dirver:

	insmod ./agtbus.ko