#include "msgdma.hpp"
#include <sys/unistd.h>
#include <altera_msgdma_descriptor_regs.h>
#include <altera_msgdma_csr_regs.h>
#include <altera_msgdma_response_regs.h>
#include <driver/critical_section.hpp>

bool MsgdmaTransmitDescriptor::transmitAsync(alt_msgdma_dev *dev) const {
    // ディスクリプタFIFOに空きが無いがあることを確認する
    if (IORD_ALTERA_MSGDMA_CSR_STATUS(dev->csr_base) & ALTERA_MSGDMA_CSR_DESCRIPTOR_BUFFER_FULL_MASK) {
        return false;
    }

    {
        CriticalSection cs;

        // ディスクリプタFIFOに空きが無いがあることを再び確認する
        if (IORD_ALTERA_MSGDMA_CSR_STATUS(dev->csr_base) & ALTERA_MSGDMA_CSR_DESCRIPTOR_BUFFER_FULL_MASK) {
            return false;
        }

        // ディスパッチャが次のディスクリプタを読み込むのを停止する
        IOWR_ALTERA_MSGDMA_CSR_CONTROL(dev->csr_base, ALTERA_MSGDMA_CSR_STOP_DESCRIPTORS_MASK);

        // IRQフラグをクリアする
        IOWR_ALTERA_MSGDMA_CSR_STATUS(dev->csr_base, ALTERA_MSGDMA_CSR_IRQ_SET_MASK);

        // ディスクリプタFIFOに書き込む
        IOWR_ALTERA_MSGDMA_DESCRIPTOR_READ_ADDRESS(dev->descriptor_base, reinterpret_cast<alt_u32>(_read_address));
        IOWR_ALTERA_MSGDMA_DESCRIPTOR_WRITE_ADDRESS(dev->descriptor_base, reinterpret_cast<alt_u32>(_write_address));
        IOWR_ALTERA_MSGDMA_DESCRIPTOR_LENGTH(dev->descriptor_base, _transfer_length);
        IOWR_ALTERA_MSGDMA_DESCRIPTOR_CONTROL_STANDARD(dev->descriptor_base, _control);

        // CONTROLレジスタを再設定する
        alt_u32 control = dev->control;
        if (dev->callback == nullptr) {
            control |= ALTERA_MSGDMA_CSR_STOP_ON_ERROR_MASK;
            control &= (~ALTERA_MSGDMA_CSR_STOP_DESCRIPTORS_MASK | ALTERA_MSGDMA_CSR_GLOBAL_INTERRUPT_MASK);
        }
        else {
            control |= ALTERA_MSGDMA_CSR_STOP_ON_ERROR_MASK | ALTERA_MSGDMA_CSR_GLOBAL_INTERRUPT_MASK;
            control &= ~ALTERA_MSGDMA_CSR_STOP_DESCRIPTORS_MASK;
        }
        IOWR_ALTERA_MSGDMA_CSR_CONTROL(dev->csr_base, control);
    }

    return true;
}
