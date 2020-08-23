from pyDAPLink.interface import HidApiUSB
from pyDAPLink.daplink import DAPLinkCore
from pyDAPLink import AP_REG, DP_REG
from pyDAPLink.daplink import CMSIS_DAP
import logging
import time
from intelhex import IntelHex

logging.basicConfig(level=logging.DEBUG)

FLASH_BASE = 0x0
FLASH_SIZE = 0x20000
PAGE_SIZE = 256
TOTAL_ROWS = FLASH_SIZE / PAGE_SIZE



#hex = IntelHex(r"Phoenix_CYPD5125-40LQXI.cydsn\noboot.cydsn\CortexM0\ARM_GCC_541\Release\noboot.hex")
hex = IntelHex(r"noboot.hex")
bin = hex.tobinarray(start=FLASH_BASE, size=FLASH_SIZE)



CPUSS_SYSREQ = 0x40100004
CPUSS_SYSARG = 0x40100008

SROM_KEY1 = 0xB6
SROM_KEY2 = 0xD3
SROM_SYSREQ_BIT = 0x80000000
SROM_PRIVILEGED_BIT = 0x10000000
SROM_STATUS_SUCCEEDED = 0xA0000000

SROM_CMD_GET_SILICON_ID = 0x00
SROM_CMD_LOAD_LATCH = 0x04
SROM_CMD_PROGRAM_ROW = 0x06
SROM_CMD_ERASE_ALL = 0x0A
SROM_CMD_CHECKSUM = 0x0B
SROM_CMD_WRITE_PROTECTION = 0x0D
SROM_CMD_SET_IMO_48_MHz = 0x15
SROM_CMD_WRITE_SFLASH_ROW = 0x18

SRAM_PARAMS_BASE = 0x20000100



def writeIO(daplink, addr, data):
    daplink._write((0 << 1) | (1 << 0) | AP_REG['TAR'], addr)
    daplink._write((0 << 1) | (1 << 0) | AP_REG['DRW'], data)

def readIO(daplink, addr):
    daplink._write((0 << 1) | (1 << 0) | AP_REG['TAR'], addr)
    daplink._write((1 << 1) | (1 << 0) | AP_REG['DRW'])
    daplink._read(4, lambda resp: (resp[0] << 0) | (resp[1] << 8) | (resp[2] << 16) | (resp[3] << 24))
    return daplink.flush()[0]
    
def waitSysCall(daplink):
    for i in range(100):
        time.sleep(0.01)
        result = readIO(daplink, CPUSS_SYSREQ)
        if (result & (SROM_PRIVILEGED_BIT | SROM_SYSREQ_BIT)) == 0:
            return True
    print("System call was timeout (0x%x)" % result)
    return False





interfaces = HidApiUSB.getConnectedInterfaces(0x03EB, 0x2141) # Atmel-ICE
#interfaces = HidApiUSB.getConnectedInterfaces(0xC251, 0xF001) # LPC Link 2
if len(interfaces) == 0:
    print("There is no CMSIS-DAP")
    exit()
    
interface = interfaces[0]
interface.open()

# Hard Reset
protocol = CMSIS_DAP(interface)
protocol.connect()
protocol.setSWJClock(1000000)
protocol.transferConfigure()
protocol.setSWJPins(0, 'nRESET')
time.sleep(0.1)
protocol.setSWJPins(0x80, 'nRESET')
protocol.disconnect()
protocol = None

# Start DAPLink
daplink = DAPLinkCore(interface)
daplink.init()
print("Initialized")

# Configure Debug Port
daplink.writeDP(DP_REG['CTRL_STAT'], 0x54000000)
daplink.writeDP(DP_REG['SELECT'], 0x00000000)
daplink.writeAP(AP_REG['CSW'], 0x00000002)

# Enable debug, and halt the CPU
writeIO(daplink, 0xE000EDF0, 0xA05F0003)

# Enable Breakpoint unit
writeIO(daplink, 0xE0002000, 0x00000003)

# Get address at reset vector
result = readIO(daplink, 0x00000004)

# Map the address bits to the breakpoint compare register
# bit map, set the enable breakpoint bit, and the match bits
reset_address = (result & 0x1FFFFFFC) | 0xC0000001

# Update the breakpoint compare register
writeIO(daplink, 0xE0002008, reset_address)
print("Reset Address is 0x%x" % reset_address)

# Issue software reset
writeIO(daplink, 0xE000ED0C, 0x05FA0004)

# Restart DAPLink
try:
    daplink.uninit()
except Exception:
    pass # Ignore exception becase software reset causes SWD Fault
daplink._protocol.disconnect()
time.sleep(0.01)
daplink = DAPLinkCore(interface)
daplink.init()

# Configure Debug Port
daplink.writeDP(DP_REG['CTRL_STAT'], 0x54000000)
daplink.writeDP(DP_REG['SELECT'], 0x00000000)
daplink.writeAP(AP_REG['CSW'], 0x00000002)

# Verify the debug enable, cpu halt bits are set
result = readIO(daplink, 0xE000EDF0)
if (result & 0x00000003) != 0x00000003:
    print("CPU halt bits are not set (0x%x)" % result)
    exit()

# Load infinite for loop code in SRAM address 0x20000300
writeIO(daplink, 0x20000300, 0xE7FEE7FE)

# Load PC with address of infinite for loop SRAM address with thumb bit (bit 0) set
writeIO(daplink, 0xE000EDF8, 0x20000301)
writeIO(daplink, 0xE000EDF4, 0x0001000F)

# Load SP with top of SRAM address - Set for minimum SRAM size devices (2 KB size)
writeIO(daplink, 0xE000EDF8, 0x20000800)
writeIO(daplink, 0xE000EDF4, 0x00010011)

# Read xPSR register, set the thumb bit, and restore modified value to xPSR register
writeIO(daplink, 0xE000EDF4, 0x00000010)
result = readIO(daplink, 0xE000EDF8)
psr_reg_val = result | 0x01000000;
writeIO(daplink, 0xE000EDF8, psr_reg_val)
writeIO(daplink, 0xE000EDF4, 0x00010010)

# Disable Breakpoint unit
writeIO(daplink, 0xE0002000, 0x00000002)

# Unhalt CPU
writeIO(daplink, 0xE000EDF0, 0xA05F0001)

# The following SROM call is not required for some targets.
# Refer to Table 1-1 on page 5 to determine whether this call is required for your device.
# Set "IMO = 48 MHz" to enable Erase/Program/Write Flash operations.
params = (SROM_KEY1 << 0) + ((SROM_KEY2 + SROM_CMD_SET_IMO_48_MHz) << 8)

# Write Params in CPUSS_SYSARG
writeIO(daplink, CPUSS_SYSARG, params);
writeIO(daplink, CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_SET_IMO_48_MHz) # Request SROM call
if not waitSysCall(daplink):
    exit()

# Erase Flash
print("Erasing flash...")

params = (SROM_KEY1 << 0) + ((SROM_KEY2 + SROM_CMD_ERASE_ALL) << 8)
writeIO(daplink, SRAM_PARAMS_BASE + 0x00, params)
writeIO(daplink, CPUSS_SYSARG, SRAM_PARAMS_BASE)
writeIO(daplink, CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_ERASE_ALL)
if not waitSysCall(daplink):
    print("Failed to erasing flash")
    exit()

# Checksum Privileged
params = (SROM_KEY1 << 0) + ((SROM_KEY2+SROM_CMD_CHECKSUM) << 8) + ((0x0000 & 0x00FF) << 16) + ((0x8000 & 0xFF00) << 16)
writeIO(daplink, CPUSS_SYSARG, params)
writeIO(daplink, CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_CHECKSUM)
if not waitSysCall(daplink):
    print("Failed to get checksum")
    exit()
result = readIO(daplink, CPUSS_SYSARG)
checksum_privileged = (result & 0x0FFFFFFF) # 28-bit checksum
print("Checksum Privileged is %X" % checksum_privileged)

# Program Flash
print("Programming flash...")
for row_index in range(TOTAL_ROWS):
    checksum = 0
    bits = 0
    offset = PAGE_SIZE * row_index
    for i in range(PAGE_SIZE / 4):
        word = bin[offset + 4 * i] | (bin[offset + 4 * i + 1] << 8) | (bin[offset + 4 * i + 2] << 16) | (bin[offset + 4 * i + 3] << 24)
        checksum += word
        bits |= word
    if bits == 0:
        print("Row number %d was skipped" % row_index)
        continue
    
    # Load Latch
    macro_id = 0
    param1 = (SROM_KEY1 << 0) + ((SROM_KEY2 + SROM_CMD_LOAD_LATCH) << 8) + (0x00 << 16) + (macro_id << 24)
    param2 = PAGE_SIZE - 1
    writeIO(daplink, SRAM_PARAMS_BASE + 0x00, param1);
    writeIO(daplink, SRAM_PARAMS_BASE + 0x04, param2);
    for i in range(PAGE_SIZE / 4):
        word = bin[offset + 4 * i] | (bin[offset + 4 * i + 1] << 8) | (bin[offset + 4 * i + 2] << 16) | (bin[offset + 4 * i + 3] << 24)
        writeIO(daplink, SRAM_PARAMS_BASE + 0x08 + 4 * i, word);
    writeIO(daplink, CPUSS_SYSARG, SRAM_PARAMS_BASE)
    writeIO(daplink, CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_LOAD_LATCH)
    if not waitSysCall(daplink):
        print("Failed to loading latch at row number %d" % row_index)
        exit()
        
    # Program Row
    params = (SROM_KEY1 << 0) + ((SROM_KEY2 + SROM_CMD_PROGRAM_ROW) << 8) + ((row_index & 0x00FF) << 16) + ((row_index & 0xFF00) << 16)
    writeIO(daplink, SRAM_PARAMS_BASE + 0x00, params)
    writeIO(daplink, CPUSS_SYSARG, SRAM_PARAMS_BASE)
    writeIO(daplink, CPUSS_SYSREQ, SROM_SYSREQ_BIT | SROM_CMD_PROGRAM_ROW)
    if not waitSysCall(daplink):
        print("Failed to programming flash at row number %d" % row_index)
        exit()
    
    print("Row number %d was programmed" % row_index)

# Verify Flash
print("Verifying flash...")
for i in range(FLASH_SIZE / 4):
    written_word = readIO(daplink, FLASH_BASE + 4 * i)
    hex_word = bin[4 * i] | (bin[4 * i + 1] << 8) | (bin[4 * i + 2] << 16) | (bin[4 * i + 3] << 24)
    if (written_word != hex_word):
        print("Verify error at %d" % (4 * i))
        break

print("Programming complete")
daplink.uninit()
interface.close()
