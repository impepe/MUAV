obj :=
obj += flight_control.c
obj += bmi088/my_bmi088.c
obj += bmi088/common.c
obj += bmi088/bmi08a.c
obj += bmi088/bmi08xa.c
obj += bmi088/bmi08g.c
obj += source/controller.c

start: $(obj)
	gcc $(obj) -lwiringPi -lm -o run
