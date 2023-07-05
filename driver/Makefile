obj-m := piradio0.o

MY_CFLAGS += -g -DDEBUG
ccflags-y += ${MY_CFLAGS}
piradio0-objs := piradio0_main.o piradio_config.o piradio_rx.o piradio_tx.o piradio_probes.o

SRC := $(shell pwd)

all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

