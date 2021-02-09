#pragma once

#include <string>
#include <stdint.h>
#include <mutex>
#include <termios.h>

class Uart
{
public:
    Uart() {}

    ~Uart()
    {
        Close();
    }

    // TTYデバイスを開く
    // 初期設定はパリティビット無し、ストップビット1、ボーレートはデフォルト値
    bool Open(const std::string &device_name);

    bool IsOpened(void) const
    {
        return (_Fd != -1);
    }

    void Close();

    // ボーレートを設定する
    // B9600, B19200, B38400, etc. といったマクロで定義されている値を指定する
    bool SetBaudrate(speed_t baudrate);

    // パリティビットを設定する
    bool SetParityBitEnabled(bool enable, bool odd);

    // ストップビットを設定する
    bool SetLongStopBitEnabled(bool enable);

    // データを送信する
    // data           : 送信するデータが格納されたバッファへのポインタ
    // length         : 送信するデータのバイト数
    // written_length : 実際に送信されたバイト数
    bool Write(const void *data, size_t length, size_t *written_length = nullptr);

    // データを受信する
    // data        : 受信したデータを格納するバッファへのポインタ
    // length      : 受信するデータのバイト数
    // read_length : 実際に受信したバイト数
    bool Read(void *data, size_t length, size_t *read_length = nullptr);

private:
    Uart(const Uart &);

    std::mutex _Mutex;
    int _Fd = -1;
};
