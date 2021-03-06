#include "adc2.hpp"
#include <sys/alt_irq.h>
#include <centralized_monitor.hpp>

bool Adc2::Initialize(void) {
    // ダミークロックを送ってI2Cバスをリセットする
    StartBusResetAsync();
    AwaitComplete();

    // ADS1015のアドレスを設定する
    I2CM_SetDeviceAddress(I2C_BASE, I2C_ADDRESS);

    // 試しにレジスタを読み出して応答を確かめる
    uint16_t config;
    if (ReadRegister(ADS1015_CONFIGURATION, &config) == false) {
        return false;
    }

    return true;
}

void Adc2::Start(void) {
    // 割り込みを有効化する
    I2CM_ClearComplete(I2C_BASE);
    alt_ic_isr_register(I2C_IC_ID, I2C_IRQ, Handler, nullptr, nullptr);

    // 最初の変換を開始する
    StartConversionAsync(0);
}

void Adc2::StartConversionAsync(int sequence) {
    if ((uint32_t)NUMBER_OF_SEQUENCE <= (uint32_t)sequence) {
        sequence = 0;
    }
    _State = STATE_WriteConfig;
    _Sequence = sequence;
    if (sequence == 0) {
        // 48V電源の出力電圧測定
        ConvertAsync(MUX_AIN3_to_AIN2, FSR_4096mV);

    }
    else {
        // ドリブルモーターの電流測定
        ConvertAsync(MUX_AIN1_to_AIN0, FSR_256mV);
    }
}

bool Adc2::ReadRegister(int address, uint16_t *value) {
    I2CM_ReadRegister2Byte(I2C_BASE, address);
    AwaitComplete();
    if (I2CM_IsAcked(I2C_BASE)) {
        uint16_t rxdata = I2CM_GetReadResult2Byte(I2C_BASE);
        *value = (rxdata << 8) | (rxdata >> 8);
        return true;
    }
    return false;
}

bool Adc2::WriteRegister(int address, uint16_t value) {
    int txdata = (value >> 8) | (value << 8);
    I2CM_WriteRegister2Byte(I2C_BASE, address, txdata);
    AwaitComplete();
    return I2CM_IsAcked(I2C_BASE);
}

void Adc2::Handler(void *context) {
    I2CM_ClearComplete(I2C_BASE);
    if (I2CM_IsAcked(I2C_BASE) == false) {
        goto error;
    }
    switch (_State) {
    case STATE_WriteConfig: {
        // 次はCONFIGURATIONレジスタをポーリングして変換の終了を待つ
        _State = STATE_PollConfig;
        I2CM_ReadRegister2Byte(I2C_BASE, ADS1015_CONFIGURATION);
        break;
    }
    case STATE_PollConfig: {
        uint16_t rxdata = I2CM_GetReadResult2Byte(I2C_BASE);
        uint16_t config = (rxdata << 8) | (rxdata >> 8);
        if (config & 0x8000) {
            // 変換結果を読み出す
            _State = STATE_ReadResult;
            I2CM_ReadRegister2Byte(I2C_BASE, ADS1015_CONVERSION_DATA);
        }
        else {
            // ポーリングを続ける
            I2CM_ReadRegister2Byte(I2C_BASE, ADS1015_CONFIGURATION);
        }
        break;
    }
    case STATE_ReadResult: {
        uint16_t rxdata = I2CM_GetReadResult2Byte(I2C_BASE);
        int16_t result = (rxdata << 8) | (rxdata >> 8);
        if (_Sequence == 0) {
            float value = static_cast<int>(result) * (1.0f / 32768.0f * 4.096f * 21.0f); // 分解能 42mV;
            _Result[0] = fmaxf(0.0f, fminf(value, 65.535f));
        }
        else {
            float value = static_cast<int>(result) * (1.0f / 32768.0f * 0.256f / 0.01f); // 分解能 12.5mA
            _Result[1] = fmaxf(0.0f, fminf(value, 65.535f));
            CentralizedMonitor::Adc2Callback();
        }
        _Valid = true;

        // 次の変換を開始する
        StartConversionAsync(_Sequence + 1);
        break;
    }
    case STATE_BusReset: {
        // 新たな変換を開始する
        StartConversionAsync(0);
    }
    default:
        goto error;
    }
    return;
    error:
    for (uint32_t i = 0; i < NUMBER_OF_SEQUENCE; i++) {
        _Result[i] = 0;
    }
    _Valid = false;
    _State = STATE_BusReset;
    StartBusResetAsync();
}

Adc2::STATE_t Adc2::_State = Adc2::STATE_WriteConfig;
int Adc2::_Sequence = 0;
float Adc2::_Result[Adc2::NUMBER_OF_SEQUENCE];
bool Adc2::_Valid = false;
