source [find tcl/interface/jlink.cfg]
transport select swd
source [find tcl/target/swj-dp.tcl]
source firmware.tcl

set _CHIPNAME cypd5
set _ENDIAN little

swj_newdap $_CHIPNAME cpu -irlen 4 -ircapture 0x1 -irmask 0xf
dap create $_CHIPNAME.dap -chain-position $_CHIPNAME.cpu

set _TARGETNAME $_CHIPNAME.cpu
target create $_TARGETNAME cortex_m -endian $_ENDIAN -dap $_CHIPNAME.dap
adapter speed 1000

init

adapter srst pulse_width 100
reset_config srst_only
adapter assert srst
sleep 100
adapter deassert srst
sleep 100

# 定数を定義する
set FLASH_BASE 0x0
set FLASH_SIZE 0x20000
set PAGE_SIZE 256
set TOTAL_ROWS [expr $FLASH_SIZE / $PAGE_SIZE]

set CPUSS_SYSREQ 0x40100004
set CPUSS_SYSARG 0x40100008

set SROM_KEY1 0xB6
set SROM_KEY2 0xD3
set SROM_SYSREQ_BIT 0x80000000
set SROM_PRIVILEGED_BIT 0x10000000
set SROM_STATUS_SUCCEEDED 0xA0000000

set SROM_CMD_GET_SILICON_ID 0x00
set SROM_CMD_LOAD_LATCH 0x04
set SROM_CMD_PROGRAM_ROW 0x06
set SROM_CMD_ERASE_ALL 0x0A
set SROM_CMD_CHECKSUM 0x0B
set SROM_CMD_WRITE_PROTECTION 0x0D
set SROM_CMD_SET_IMO_48_MHz 0x15
set SROM_CMD_WRITE_SFLASH_ROW 0x18

set SRAM_PARAMS_BASE 0x20000100


# CPUを止める関数
proc cpu_stop {} {
	# Enable debug, and halt the CPU
	mww 0xE000EDF0 0xA05F0003

	# Enable Breakpoint unit
	mww 0xE0002000 0x00000003

	# Get address of reset vector
	mem2array result 32 0x00000004 1

	# Map the address bits to the breakpoint compare register
	# bit map, set the enable breakpoint bit, and the match bits
	set reset_address [expr ($result(0) & 0x1FFFFFFC) | 0xC0000001]

	# Update the breakpoint compare register
	mww 0xE0002008 $reset_address
	puts [format "Reset Address is 0x%X" $reset_address]

	# Issue software reset
	mww 0xE000ED0C 0x05FA0004
}

# CPUを無限ループさせる関数
proc cpu_infiniteloop {} {
	# Verify the debug enable, cpu halt bits are set
	mem2array result 32 0xE000EDF0 1
	if {($result(0) & 0x00000003) != 0x00000003} {
		puts ["CPU halt bits are not set (0x%X)" $result(0)]
		exit
	}

	# Load infinite for loop code in SRAM address 0x20000300
	mww 0x20000300 0xE7FEE7FE

	# Load PC with address of infinite for loop SRAM address with thumb bit (bit 0) set
	mww 0xE000EDF8 0x20000301
	mww 0xE000EDF4 0x0001000F

	# Load SP with top of SRAM address - Set for minimum SRAM size devices (2 KB size)
	mww 0xE000EDF8 0x20000800
	mww 0xE000EDF4 0x00010011

	# Read xPSR register, set the thumb bit, and restore modified value to xPSR register
	mww 0xE000EDF4 0x00000010
	mem2array result 32 0xE000EDF8 1
	mww 0xE000EDF8 [expr $result(0) | 0x01000000]
	mww 0xE000EDF4 0x00010010

	# Disable Breakpoint unit
	mww 0xE0002000 0x00000002

	# Unhalt CPU
	mww 0xE000EDF0 0xA05F0001
}

# システムコールの完了を待つ
proc wait_syscall {} {
    global CPUSS_SYSREQ SROM_PRIVILEGED_BIT SROM_SYSREQ_BIT
    for {set i 0} {$i < 100} {incr i} {
        sleep 10
        mem2array result 32 $CPUSS_SYSREQ 1
        if {($result(0) & ($SROM_PRIVILEGED_BIT | $SROM_SYSREQ_BIT)) == 0} {
            return 1
        }
    }
    puts [format "System call was timeout (0x%x)" $result]
    return 0
}

# 発振器を起動する
proc start_oscillator {} {
    global SROM_KEY1 SROM_KEY2 SROM_CMD_SET_IMO_48_MHz CPUSS_SYSARG CPUSS_SYSREQ SROM_SYSREQ_BIT
    
    # The following SROM call is not required for some targets.
    # Refer to Table 1-1 on page 5 to determine whether this call is required for your device.
    # Set "IMO = 48 MHz" to enable Erase/Program/Write Flash operations.
    set params [expr ($SROM_KEY1 << 0) + (($SROM_KEY2 + $SROM_CMD_SET_IMO_48_MHz) << 8)]
    
    # Write Params in CPUSS_SYSARG
    mww $CPUSS_SYSARG $params
    mww $CPUSS_SYSREQ [expr $SROM_SYSREQ_BIT | $SROM_CMD_SET_IMO_48_MHz]
    return [wait_syscall]
}

# フラッシュのprivileged行のチェックサムを取得する
proc get_checksum {} {
    global SROM_KEY1 SROM_KEY2 SROM_CMD_CHECKSUM CPUSS_SYSARG CPUSS_SYSREQ SROM_SYSREQ_BIT
    set params [expr ($SROM_KEY1 << 0) + (($SROM_KEY2+$SROM_CMD_CHECKSUM) << 8) + ((0x0000 & 0x00FF) << 16) + ((0x8000 & 0xFF00) << 16)]
    mww $CPUSS_SYSARG $params
    mww $CPUSS_SYSREQ [expr $SROM_SYSREQ_BIT | $SROM_CMD_CHECKSUM]
    if {[wait_syscall] == 0} {
        puts "Failed to get checksum"
        return -1
    }
    mem2array result 32 $CPUSS_SYSARG 1
    return [expr ($result(0) & 0x0FFFFFFF)]
}

# フラッシュを消去する
proc erase_flash {} {
    global SROM_KEY1 SROM_KEY2 SROM_CMD_ERASE_ALL SRAM_PARAMS_BASE CPUSS_SYSARG CPUSS_SYSREQ SROM_SYSREQ_BIT
    set params [expr ($SROM_KEY1 << 0) + (($SROM_KEY2 + $SROM_CMD_ERASE_ALL) << 8)]
    mww [expr $SRAM_PARAMS_BASE + 0x00] $params
    mww $CPUSS_SYSARG $SRAM_PARAMS_BASE
    mww $CPUSS_SYSREQ [expr $SROM_SYSREQ_BIT | $SROM_CMD_ERASE_ALL]
    return [wait_syscall]
}

# フラッシュに1ページ書き込む
proc program_flash_page {row_index} {
    global PAGE_SIZE FIRMWARE SROM_KEY1 SROM_KEY2 SROM_CMD_LOAD_LATCH SRAM_PARAMS_BASE CPUSS_SYSARG CPUSS_SYSREQ SROM_SYSREQ_BIT SROM_CMD_PROGRAM_ROW

    set offset [expr $PAGE_SIZE / 4 * $row_index]
    set bits 0
    set count [expr $PAGE_SIZE / 4]
    for {set i 0} {$i < $count} {incr i} {
        set word [lindex $FIRMWARE [expr $offset + $i]]
        set bits [expr $bits | $word]
    }
    if {$bits == 0} {
        puts [format "Row index %d was skipped" $row_index]
        return 1
    }
    
    # Load Latch
    set param1 [expr ($SROM_KEY1 << 0) + (($SROM_KEY2 + $SROM_CMD_LOAD_LATCH) << 8) + (0x00 << 16) + (0 << 24)]
    set param2 [expr $PAGE_SIZE - 1]
    mww [expr $SRAM_PARAMS_BASE + 0x00] $param1
    mww [expr $SRAM_PARAMS_BASE + 0x04] $param2
    for {set i 0} {$i < $count} {incr i} {
        set word [lindex $FIRMWARE [expr $offset + $i]]
        mww [expr $SRAM_PARAMS_BASE + 0x08 + 4 * $i] $word
    }
    mww $CPUSS_SYSARG $SRAM_PARAMS_BASE
    mww $CPUSS_SYSREQ [expr $SROM_SYSREQ_BIT | $SROM_CMD_LOAD_LATCH]
    if {[wait_syscall] == 0} {
        puts [format "Failed to loading latch at row index %d" $row_index]
        return 0
    }
    
    # Program Row
    set params [expr ($SROM_KEY1 << 0) + (($SROM_KEY2 + $SROM_CMD_PROGRAM_ROW) << 8) + (($row_index & 0x00FF) << 16) + (($row_index & 0xFF00) << 16)]
    mww [expr $SRAM_PARAMS_BASE + 0x00] $params
    mww $CPUSS_SYSARG $SRAM_PARAMS_BASE
    mww $CPUSS_SYSREQ [expr $SROM_SYSREQ_BIT | $SROM_CMD_PROGRAM_ROW]
    if {[wait_syscall] == 0} {
        puts [format "Failed to programming flash at row index %d" $row_index]
        return 0
    }
    puts [format "Row index %d was programmed" $row_index]
    return 1
}

# フラッシュに書き込んだ内容を比較する関数
proc verify_flash {} {
	global FLASH_BASE FLASH_SIZE FIRMWARE
    set count [expr $FLASH_SIZE / 4]
	mem2array result_array 32 $FLASH_BASE $count
    set result [lrepeat $FLASH_SIZE 0]
    for {set i 0} {$i < $count} {incr i} {
        if {$result_array($i) != [lindex $FIRMWARE $i]} {
            return [$i * 4]
        }
    }
    # すべてのデータが一致した
    return -1
}

puts "Halting CPU..."
cpu_stop
puts "OK"

sleep 100

puts "Falling CPU into an infinite loop..."
cpu_infiniteloop
puts "OK"

puts "Starting 48MHz Oscillator..."
if {[start_oscillator] != 0} {
    puts "OK"
} else {
    puts "Failed"
    exit
}

puts "Erasing flash..."
if {[erase_flash] != 0} {
    puts "OK"
} else {
    puts "Failed"
}

puts "Checking Privileged Checksum..."
set checksum [get_checksum]
if {$checksum == 0} {
    puts "OK"
} else {
    puts [format "Failed (%07X)" $checksum]
}

puts "Programming flash..."
for {set i 0} {$i < $TOTAL_ROWS} {incr i} {
    if {[program_flash_page $i] == 0} {
        exit
    }
}
puts "OK"

puts "Verifying flash..."
set mismatch [verify_flash]
if {$mismatch < 0} {
    puts "OK"
} else {
    puts [format "Failed (index:0x%X)" $mismatch]
}

exit
