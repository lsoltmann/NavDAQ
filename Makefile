CC = g++
NAVIO = /home/pi/Navio/C++/Navio
LIBS = /home/pi/Libraries

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)
INCLUDES = -I ../.. -I$(PIGPIO_PATH)

all:
	$(CC) $(INCLUDES) $(LIB) NavDAQ.cpp \
        $(NAVIO)/MPU9250.cpp \
        $(NAVIO)/ADS1115.cpp \
        $(NAVIO)/MS5611.cpp \
	$(NAVIO)/PCA9685.cpp \
	$(NAVIO)/gpio.cpp \
	$(LIBS)/UbloxGPS.cpp \
	$(LIBS)/MS5805.cpp \
	$(LIBS)/MS4515.cpp \
	$(LIBS)/PID.cpp \
	$(LIBS)/SSC005D.cpp \
	$(LIBS)/readConfig.cpp \
        $(NAVIO)/I2Cdev.cpp -o NavDAQ -lrt -pthread -lpigpio -lwiringPi

clean:
	rm NavDAQ

