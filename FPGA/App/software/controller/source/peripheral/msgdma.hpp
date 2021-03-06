#pragma once

#include <altera_msgdma.h>

/**
 * mSGDMAのディスクリプタ
 */
class MsgdmaTransmitDescriptor {
public:
    /**
     * コンストラクタ
     * @param data　送りたいデータ
     * @param channel チャンネル番号
     */
    template<class T>
    constexpr MsgdmaTransmitDescriptor(const T &data, int channel) :
        _ReadAddress(&data),
        _WriteAddress(nullptr),
        _TransferLength(sizeof(T)),
        _Control(ALTERA_MSGDMA_DESCRIPTOR_CONTROL_GO_MASK
            | ALTERA_MSGDMA_DESCRIPTOR_CONTROL_GENERATE_SOP_MASK
            | ALTERA_MSGDMA_DESCRIPTOR_CONTROL_GENERATE_EOP_MASK
            | static_cast<alt_u32>(channel << ALTERA_MSGDMA_DESCRIPTOR_CONTROL_TRANSMIT_CHANNEL_OFFSET)) {}

    /**
     * 非同期的に転送を開始する。
     * alt_msgdma_standard_descriptor_async_transfer()の作りが悪いので再実装した。
     * @param device mSGDMAのハンドル
     * @return　転送の開始に成功したらtrueを返す
     */
    bool TransmitAsync(alt_msgdma_dev *device) const;

private:
    const void *_ReadAddress;
    void *_WriteAddress;
    alt_u32 _TransferLength;
    alt_u32 _Control;
};
