from intelhex import IntelHex

FLASH_BASE = 0x0
FLASH_SIZE = 0x20000

hex = IntelHex(r"noboot.hex")
bin = hex.tobinarray(start=FLASH_BASE, size=FLASH_SIZE)

with open("firmware.tcl", "w") as f:
    f.write("set FIRMWARE {")
    column = 0
    for i in range(FLASH_SIZE // 4):
        word = bin[4 * i] | (bin[4 * i + 1] << 8) | (bin[4 * i + 2] << 16) | (bin[4 * i + 3] << 24)
        if column == 0:
            text = f"\n\t0x{word:08X}"
        else:
            text = f" 0x{word:08X}"
        column = (column + 1) % 16
        f.write(text)
    f.write("\n}\n")
