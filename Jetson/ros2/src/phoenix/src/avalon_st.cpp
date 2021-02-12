#include "avalon_st.hpp"

void AvalonStPacketsToBytesConverter::Convert(const std::vector<uint8_t> &payload, int channel, std::vector<uint8_t> &output) {
    if (payload.empty() == true) {
        // ペイロード無しは許されない
        return;
    }

    // push_back()を呼びうる最大回数だけoutputのバッファを先に確保しておく
    output.reserve(output.size() + payload.size() * 2 + 5);

    // SOF
    output.push_back(0x7A);

    // Channel
    if ((0 <= channel) && (channel <= 255)) {
        output.push_back(0x7C);
        if ((channel < 0x7A) || (0x7D < channel)) {
            output.push_back(static_cast<uint8_t>(channel));
        } else {
            output.push_back(0x7D);
            output.push_back(static_cast<uint8_t>(channel ^ 0x20));
        }
    }

    // Payload + EOF
    size_t end = payload.size() - 1;
    for (size_t index = 0; index <= end; index++) {
        if (index == end) {
            // 最後のペイロードバイトの前にEOF
            output.push_back(0x7B);
        }
        int data = payload[index];
        if ((data < 0x7A) || (0x7D < data)) {
            output.push_back(static_cast<uint8_t>(data));
        } else {
            output.push_back(0x7D);
            output.push_back(static_cast<uint8_t>(data ^ 0x20));
        }
    }
}
