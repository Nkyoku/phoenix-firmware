# TCL File Generated by Component Editor 20.1
# Thu Feb 11 12:01:42 JST 2021
# DO NOT MODIFY


# 
# float32to16 "FP16 to FP32 Converter" v1.0
# Fujii Naomichi 2021.02.11.12:01:42
# 
# 

# 
# request TCL package from ACDS 16.1
# 
package require -exact qsys 16.1


# 
# module float32to16
# 
set_module_property DESCRIPTION ""
set_module_property NAME float32to16
set_module_property VERSION 1.0
set_module_property INTERNAL false
set_module_property OPAQUE_ADDRESS_MAP true
set_module_property AUTHOR "Fujii Naomichi"
set_module_property DISPLAY_NAME "FP32 to FP16 Converter"
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property EDITABLE true
set_module_property REPORT_TO_TALKBACK false
set_module_property ALLOW_GREYBOX_GENERATION false
set_module_property REPORT_HIERARCHY false


# 
# file sets
# 
add_fileset QUARTUS_SYNTH QUARTUS_SYNTH "" ""
set_fileset_property QUARTUS_SYNTH TOP_LEVEL float32to16
set_fileset_property QUARTUS_SYNTH ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property QUARTUS_SYNTH ENABLE_FILE_OVERWRITE_MODE true
add_fileset_file float32to16.sv SYSTEM_VERILOG PATH float32to16.sv TOP_LEVEL_FILE

add_fileset SIM_VERILOG SIM_VERILOG "" ""
set_fileset_property SIM_VERILOG TOP_LEVEL float32to16
set_fileset_property SIM_VERILOG ENABLE_RELATIVE_INCLUDE_PATHS false
set_fileset_property SIM_VERILOG ENABLE_FILE_OVERWRITE_MODE true
add_fileset_file float32to16.sv SYSTEM_VERILOG PATH float32to16.sv TOP_LEVEL_FILE


# 
# parameters
# 


# 
# display items
# 


# 
# connection point slave
# 
add_interface slave nios_custom_instruction end
set_interface_property slave clockCycle 0
set_interface_property slave operands 1
set_interface_property slave ENABLED true
set_interface_property slave EXPORT_OF ""
set_interface_property slave PORT_NAME_MAP ""
set_interface_property slave CMSIS_SVD_VARIABLES ""
set_interface_property slave SVD_ADDRESS_GROUP ""

add_interface_port slave slave_reset reset Input 1
add_interface_port slave slave_clk clk Input 1
add_interface_port slave slave_clk_en clk_en Input 1
add_interface_port slave slave_start start Input 1
add_interface_port slave slave_done done Output 1
add_interface_port slave slave_dataa dataa Input 32
add_interface_port slave slave_result result Output 32

