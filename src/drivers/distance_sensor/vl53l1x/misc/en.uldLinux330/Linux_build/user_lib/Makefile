#CC=gcc
#CCPREFIX=/home/user/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-
CC=$(CCPREFIX)gcc


INC_CFLAGS=-I.
CORE=core
CFLAGS+=$(INC_CFLAGS) -I$(CORE) -Wall -g -D_LINUX_ 

ifdef POLLING
	CFLAGS+=-D POLLING
endif


MAIN_OBJ = main.o
CORE_OBJ =  $(CORE)/VL53L1X_api.o  \
 		$(CORE)/VL53L1X_calibration.o
PLTF_OBJ =  vl53l1_linux_platform.o


%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)


ultra_lite: $(CORE_OBJ) $(PLTF_OBJ) $(MAIN_OBJ)
	$(CC) -o $@ $^ $(CFLAGS) -lpthread

clean:
	find . -name "*.o" | xargs rm -f
	rm -f app ultra_lite

