# Error Flags
bool error_module_sleep
bool error_fpga_stop
bool error_dc48v_uv
bool error_dc48v_ov
bool[5] error_motor_oc
bool[5] error_motor_hall_sensor

# Fault Flags
bool fault_adc2_timeout
bool fault_imu_timeout
bool[5] fault_motor_ot
bool[5] fault_motor_oc
bool[5] fault_motor_load_switch
