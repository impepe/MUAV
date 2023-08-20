obj :=
obj += flight_control.c
obj += lib/bmi088/common.o
obj += lib/bmi088/bmi08a.o
obj += lib/bmi088/bmi08xa.o
obj += lib/bmi088/bmi08g.o
obj += source/nrf2.c
obj += source/filter.c
obj += source/my_bmi088.c
obj += source/controller.c
obj += source/pos_info.c
obj += source/gpio.c
obj += source/funcs.c

start: $(obj)
	g++ $(obj) -lwiringPi -lrf24 -lpthread -lm -o run
