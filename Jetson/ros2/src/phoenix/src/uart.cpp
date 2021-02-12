#include "uart.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

bool Uart::Open(const std::string &device_name)
{
    std::lock_guard<std::mutex> lock(_Mutex);
    if (_Fd != -1)
        close(_Fd);
    _Fd = open(device_name.c_str(), O_RDWR);
    if (_Fd == -1)
        return false;
    do
    {
        // 初期設定を読み込んでバイナリ送受信用にそれを編集する
        struct termios setting;
        if (tcgetattr(_Fd, &setting) != 0)
            break;
        cfmakeraw(&setting);
        setting.c_cflag |= CREAD | CLOCAL;
        setting.c_cflag &= ~CSTOPB;
        setting.c_cc[VMIN] = 0;  // 1バイトでも受信したらread()が返る
        setting.c_cc[VTIME] = 1; // read()のタイムアウトは1/10秒
        if (tcsetattr(_Fd, TCSANOW, &setting) != 0)
            break;
        return true;
    } while (false);
    Close();
    return false;
}

void Uart::Close()
{
    std::lock_guard<std::mutex> lock(_Mutex);
    if (_Fd != -1)
    {
        close(_Fd);
        _Fd = -1;
    }
}

bool Uart::SetBaudrate(speed_t baudrate)
{
    std::lock_guard<std::mutex> lock(_Mutex);
    if (!IsOpened())
        return false;
    struct termios setting;
    if (tcgetattr(_Fd, &setting) != 0)
        return false;
    if (cfsetspeed(&setting, baudrate) != 0)
        return false;
    if (tcsetattr(_Fd, TCSANOW, &setting) != 0)
        return false;
    return true;
}

bool Uart::SetParityBitEnabled(bool enable, bool odd)
{
    std::lock_guard<std::mutex> lock(_Mutex);
    if (!IsOpened())
        return false;
    struct termios setting;
    if (tcgetattr(_Fd, &setting) != 0)
        return false;
    if (enable)
    {
        setting.c_cflag |= PARENB;
        if (odd)
            setting.c_cflag |= PARODD;
        else
            setting.c_cflag &= ~PARODD;
    }
    else
    {
        setting.c_cflag &= ~PARENB;
    }
    if (tcsetattr(_Fd, TCSANOW, &setting) != 0)
        return false;
    return true;
}

bool Uart::SetLongStopBitEnabled(bool enable)
{
    std::lock_guard<std::mutex> lock(_Mutex);
    if (!IsOpened())
        return false;
    struct termios setting;
    if (tcgetattr(_Fd, &setting) != 0)
        return false;
    if (enable)
        setting.c_cflag |= CSTOPB;
    else
        setting.c_cflag &= ~CSTOPB;
    if (tcsetattr(_Fd, TCSANOW, &setting) != 0)
        return false;
    return true;
}

bool Uart::Write(const void *data, size_t length, size_t *written_length)
{
    ssize_t result = write(_Fd, data, length);
    if (result < 0)
        return false;
    if (written_length != nullptr)
        *written_length = result;
    return true;
}

bool Uart::Read(void *data, size_t length, size_t *read_length)
{
    ssize_t result = read(_Fd, data, length);
    if (result < 0)
        return false;
    if (read_length != nullptr)
        *read_length = result;
    return true;
}
