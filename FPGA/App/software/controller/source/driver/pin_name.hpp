#pragma once

enum Pio0Bit {
    Pio0Pulse1kHz = 1u << 0
};

enum Pio1Bit {
    Pio1Motor5HallW = 1u << 30,
    Pio1Motor5HallV = 1u << 29,
    Pio1Motor5HallU = 1u << 28,
    Pio1Motor4EncoderB = 1u << 27,
    Pio1Motor4EncoderA = 1u << 26,
    Pio1Motor4HallW = 1u << 25,
    Pio1Motor4HallV = 1u << 24,
    Pio1Motor4HallU = 1u << 23,
    Pio1Motor3EncoderB = 1u << 22,
    Pio1Motor3EncoderA = 1u << 21,
    Pio1Motor3HallW = 1u << 20,
    Pio1Motor3HallV = 1u << 19,
    Pio1Motor3HallU = 1u << 18,
    Pio1Motor2EncoderB = 1u << 17,
    Pio1Motor2EncoderA = 1u << 16,
    Pio1Motor2HallW = 1u << 15,
    Pio1Motor2HallV = 1u << 14,
    Pio1Motor2HallU = 1u << 13,
    Pio1Motor1EncoderB = 1u << 12,
    Pio1Motor1EncoderA = 1u << 11,
    Pio1Motor1HallW = 1u << 10,
    Pio1Motor1HallV = 1u << 9,
    Pio1Motor1HallU = 1u << 8,
    Pio1Motor5SwitchFault = 1u << 7, // 注意:反転させている (PIOのレベル割り込みはHighで反応するため)
    Pio1Motor4SwitchFault = 1u << 6, // 注意:反転させている
    Pio1Motor3SwitchFault = 1u << 5, // 注意:反転させている
    Pio1Motor2SwitchFault = 1u << 4, // 注意:反転させている
    Pio1Motor1SwitchFault = 1u << 3, // 注意:反転させている
    Pio1FpgaStop = 1u << 2,          // 注意:反転させている
    Pio1ModuleSleep = 1u << 1,       // 注意:反転させている
    Pio1FpgaMode = 1u << 0
};

enum Pio2Bit {
    Pio2Motor5SwitchEnable = 1u << 9,
    Pio2Motor4SwitchEnable = 1u << 8,
    Pio2Motor3SwitchEnable = 1u << 7,
    Pio2Motor2SwitchEnable = 1u << 6,
    Pio2Motor1SwitchEnable = 1u << 5,
    Pio2Motor5Led = 1u << 4,
    Pio2Motor4Led = 1u << 3,
    Pio2Motor3Led = 1u << 2,
    Pio2Motor2Led = 1u << 1,
    Pio2Motor1Led = 1u << 0
};
