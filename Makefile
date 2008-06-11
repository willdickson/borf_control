driver_dir = /realtime/src/motor_ctl/
shm_lib_dir = /realtime/src/shm_lib/
motor_comm_dir = /interface/motor_comm_0.1/
motor_shm_dir = /interface/motor_shm_0.1/

default: motor_ctl shm_lib

motor_ctl:
	$(MAKE) -C $(PWD)$(driver_dir)

shm_lib:
	$(MAKE) -C $(PWD)$(shm_lib_dir)

.PHONY: clean install

clean: clean
	$(MAKE) clean -C $(PWD)$(driver_dir)
	$(MAKE) clean -C $(PWD)$(shm_lib_dir) 

install:
	$(MAKE) install -C $(PWD)$(driver_dir)
	$(MAKE) install -C $(PWD)$(shm_lib_dir) 
#	$(MAKE) install -C $(PWD)$(motor_comm_dir)
#	$(MAKE) install -C $(PWD)$(motor_shm_dir)


