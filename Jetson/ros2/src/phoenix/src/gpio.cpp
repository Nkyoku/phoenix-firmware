#include "gpio.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <thread>

#include <stdio.h>

Gpio::Gpio(int pin_number) : _IsOpened(false), _PinNumber(pin_number) {
    if ((pin_number < 0) || (255 < pin_number)) {
        return;
    }

    _GpioValuePath = "/sys/class/gpio/gpio" + std::to_string(pin_number) + "/value";
    _GpioDirectionPath = "/sys/class/gpio/gpio" + std::to_string(pin_number) + "/direction";

    if (_PermissionChecked == false) {
        // アクセス権限をチェックする
        printf("Permission Check\n");
        _PermissionChecked = true;
        if ((access(ExportPath, W_OK) == 0) && (access(UnexportPath, W_OK) == 0)) {
            printf("Permission Ok\n");
            _PermissionOk = true;
        }
    }

    if (_PermissionOk == true) {
        // GPIOを有効化する
        printf("Enable GPIO %d\n", _PinNumber);
        std::ofstream export_stream(ExportPath);
        export_stream << std::to_string(_PinNumber);
        export_stream.close();

        // GPIOにアクセス可能になったか確認する
        printf("Check GPIO %d\n", _PinNumber);
        int timeout = 100;
        while (access(_GpioValuePath.c_str(), R_OK | W_OK) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (--timeout < 0) {
                printf("Timeout '%s'\n", _GpioValuePath.c_str());
                return;
            }
        }
        while (access(_GpioDirectionPath.c_str(), W_OK) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (--timeout < 0) {
                printf("Timeout '%s'\n", _GpioDirectionPath.c_str());
                return;
            }
        }

        _IsOpened = true;
    }
}

Gpio::~Gpio() {
    if (_IsOpened == false) {
        return;
    }

    // GPIOを無効化する
    printf("Disable GPIO %d\n", _PinNumber);
    std::ofstream unexport_stream(UnexportPath);
    unexport_stream << std::to_string(_PinNumber);
    unexport_stream.close();
}

void Gpio::SetOutputEnabled(bool enabled) {
    if (_IsOpened == false) {
        return;
    }
    std::ofstream direction_stream(_GpioDirectionPath);
    direction_stream << (enabled ? "out" : "in");
}

void Gpio::SetOutputValue(bool value) {
    if (_IsOpened == false) {
        return;
    }
    std::ofstream value_stream(_GpioValuePath);
    value_stream << (value ? "1" : "0");
}

bool Gpio::GetInputValue(void) {
    if (_IsOpened == false) {
        return false;
    }
    std::ifstream value_stream(_GpioValuePath);
    int result;
    value_stream >> result;
    return result != 0;
}

const char Gpio::ExportPath[] = "/sys/class/gpio/export";
const char Gpio::UnexportPath[] = "/sys/class/gpio/unexport";
bool Gpio::_PermissionChecked = false;
bool Gpio::_PermissionOk = false;

/*int main(void) {
    printf("Initialize GPIO");
    Gpio fpga_mode(Gpio::JetsonNanoModulePinGpio12);
    fpga_mode.SetOutputEnabled(true);

    for(int i = 0; i < 10; i++){
        fpga_mode.SetOutputValue(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        fpga_mode.SetOutputValue(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    return 0;
}*/
