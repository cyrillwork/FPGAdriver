obj-m = agtbus.o
#KVERSION = 4.4.0-21-generic

#LIBMOD = /lib/modules/$(KVERSION)/build
LIBMOD = /lib/modules/4.4.83-1604/build
#LIBMOD = /lib/modules/4.4.0-97-generic/build

PWD := $(shell pwd)

all:
	make -C $(LIBMOD) M=$(PWD) modules
clean:
	make -C $(LIBMOD) M=$(PWD) clean
