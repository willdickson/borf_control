local_include=$(PWD)/../../include/
PWD=$(shell pwd)
CC= gcc
CFLAGS= -DCLKDIR -fPIC -c 
#CFLAGS= -DPWM -DTRIG  -fPIC -c 
INCLUDES= -I/usr/realtime/include/
EXTRA_FLAGS= -iquote$(local_include)

libmotor_shm.so.1: motor_shm.o
	${CC} -shared -W1,-soname,$@ -o $@.0.1 $?

motor_shm.o: motor_shm.c
	${CC} ${CFLAGS} ${EXTRA_FLAGS} $? -o $@ ${INCLUDES} 

.PHONY: clean install

install:
	$(PWD)/install-libmotor_shm
clean: 
	rm libmotor_shm.so.1.0.1 motor_shm.o  


