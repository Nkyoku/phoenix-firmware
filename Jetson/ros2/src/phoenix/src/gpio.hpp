#pragma once

#include <string>

class Gpio {
public:
    /// GPIOの各ポートのオフセット
    enum Port {
        PortA = 0x0,
        PortB = 0x8,
        PortC = 0x10,
        PortD = 0x18,
        PortE = 0x20,
        PortF = 0x28,
        PortG = 0x30,
        PortH = 0x38,
        PortI = 0x40,
        PortJ = 0x48,
        PortK = 0x50,
        PortL = 0x58,
        PortM = 0x60,
        PortN = 0x68,
        PortO = 0x70,
        PortP = 0x78,
        PortQ = 0x80,
        PortR = 0x88,
        PortS = 0x90,
        PortT = 0x98,
        PortU = 0xA0,
        PortV = 0xA8,
        PortW = 0xB0,
        PortX = 0xB8,
        PortY = 0xC0,
        PortZ = 0xC8,
        PortAA = 0xD0,
        PortBB = 0xD8,
        PortCC = 0xE0,
        PortDD = 0xE8,
        PortEE = 0xF0,
        PortFF = 0xF8
    };

    /// Jetson Nanoモジュールから出ているピン名からピン番号への対応
    enum JetsonNanoModulePin {
        /// GPIO3_PCC.04 : GPIO00/USB_VBUS
        JetsonNanoModulePinGpio00 = PortCC + 4,

        /// GPIO3_PS.05 : GPIO01/CAM2_MCLK
        JetsonNanoModulePinGpio01 = PortS + 5,

        /// GPIO3_PH.06 : GPIO02
        JetsonNanoModulePinGpio02 = PortH + 6,

        /// GPIO3_PI.02 : GPIO03
        JetsonNanoModulePinGpio03 = PortI + 2,

        /// GPIO3_PI.01 : GPIO04
        JetsonNanoModulePinGpio04 = PortI + 1,

        /// GPIO3_PH.07 : GPIO05
        JetsonNanoModulePinGpio05 = PortH + 7,

        /// GPIO3_PI.00 : GPIO06
        JetsonNanoModulePinGpio06 = PortI + 0,

        /// GPIO3_PV.00 : GPIO07/PWM
        JetsonNanoModulePinGpio07 = PortV + 0,

        /// GPIO3_PZ.02 : GPIO08/SDMMC_CD
        JetsonNanoModulePinGpio08 = PortZ + 2,

        /// GPIO3_PBB.00 : GPIO09/AUD_MCLK
        JetsonNanoModulePinGpio09 = PortBB + 0,

        /// GPIO3_PV.01 : GPIO10
        JetsonNanoModulePinGpio10 = PortV + 1,

        /// GPIO3_PZ.00 : GPIO11/CAM3_MCLK
        JetsonNanoModulePinGpio11 = PortZ + 0,

        /// GPIO3_PY.02 : GPIO12
        JetsonNanoModulePinGpio12 = PortY + 2,

        /// GPIO3_PE.06 : GPIO13/PWM
        JetsonNanoModulePinGpio13 = PortE + 6,

        /// GPIO3_PE.07 : GPIO14/FAN_PWM
        JetsonNanoModulePinGpio14 = PortE + 7,

        /// GPIO3_PD.01
        JetsonNanoModulePinUart0Tx = PortD + 1,

        /// GPIO3_PD.02
        JetsonNanoModulePinUart0Rx = PortD + 2,

        /// GPIO3_PD.03
        JetsonNanoModulePinUart0Rts = PortD + 3,

        /// GPIO3_PD.04
        JetsonNanoModulePinUart0Cts = PortD + 4,

        /// GPIO3_PG.00
        JetsonNanoModulePinUart1Tx = PortG + 0,

        /// GPIO3_PG.01
        JetsonNanoModulePinUart1Rx = PortG + 1,

        /// GPIO3_PG.02
        JetsonNanoModulePinUart1Rts = PortG + 2,

        /// GPIO3_PG.03
        JetsonNanoModulePinUart1Cts = PortG + 3,

        /// GPIO3_PC.03
        JetsonNanoModulePinSpi0Cs0 = PortC + 3,

        /// GPIO3_PC.04
        JetsonNanoModulePinSpi0Cs1 = PortC + 4,

        /// GPIO3_PC.02
        JetsonNanoModulePinSpi0Sck = PortC + 2,

        /// GPIO3_PC.00
        JetsonNanoModulePinSpi0Mosi = PortC + 0,

        /// GPIO3_PC.01
        JetsonNanoModulePinSpi0Miso = PortC + 1,

        /// GPIO3_PB.07
        JetsonNanoModulePinSpi1Cs0 = PortB + 7,

        /// GPIO3_PDD.00
        JetsonNanoModulePinSpi1Cs1 = PortDD + 0,

        /// GPIO3_PB.06
        JetsonNanoModulePinSpi1Sck = PortB + 6,

        /// GPIO3_PB.04
        JetsonNanoModulePinSpi1Mosi = PortB + 4,

        /// GPIO3_PB.05
        JetsonNanoModulePinSpi1Miso = PortB + 5,

        /// GPIO3_PJ.04
        JetsonNanoModulePinI2s0Fs = PortJ + 4,

        /// GPIO3_PJ.07
        JetsonNanoModulePinI2s0Sclk = PortJ + 7,

        /// GPIO3_PJ.06
        JetsonNanoModulePinI2s0Dout = PortJ + 6,

        /// GPIO3_PJ.05
        JetsonNanoModulePinI2s0Din = PortJ + 5,

        /// GPIO3_PE.00
        JetsonNanoModulePinI2s1Fs = PortE + 0,

        /// GPIO3_PE.03
        JetsonNanoModulePinI2s1Sclk = PortE + 3,

        /// GPIO3_PE.02
        JetsonNanoModulePinI2s1Dout = PortE + 2,

        /// GPIO3_PE.01
        JetsonNanoModulePinI2s1Din = PortE + 1,
    };

    /**
     * コンストラクタ
     * @param pin_number GPIOのピン番号
     */
    Gpio(int pin_number);

    ~Gpio();

    /**
     * GPIO操作が可能か取得する
     */
    bool IsOpened(void) const {
        return _IsOpened;
    }

    /**
     * GPIOの出力ドライバを有効にする
     * @param enabled trueなら出力、falseなら入力になる
     */
    void SetOutputEnabled(bool enabled);

    /**
     * 出力値を設定する
     * @param value trueならH、falseならLを出力する
     */
    void SetOutputValue(bool value);

    /**
     * 入力値を取得する
     * @return trueならH、falseならLが入力されている
     */
    bool GetInputValue(void);

private:    
    std::string GetBasePath(void){
        return std::string("/sys/class/gpio/gpio") + std::to_string(_PinNumber);
    }

    static const char ExportPath[];
    static const char UnexportPath[];

    static bool _PermissionChecked;
    static bool _PermissionOk;
    
    bool _IsOpened;
    int _PinNumber;
    std::string _GpioValuePath;
    std::string _GpioDirectionPath;
};
