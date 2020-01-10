KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

obj-m += VirtualSerialPort.o

all:
# use Tab
	$(MAKE) -C $(KDIR) M=$(PWD) modules
	
clean:
# use Tab
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
