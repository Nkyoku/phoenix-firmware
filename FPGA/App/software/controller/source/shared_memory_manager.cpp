#include "shared_memory_manager.hpp"

bool SharedMemory::UpdateParameters(void){
    // 共有メモリーからパラメータを一時的にコピーする
    SharedMemory_t *shared_memory = GetNonCachedSharedMemory();
    uint32_t tail_checksum = shared_memory->TailChecksum;
    SharedMemory_t::Parameters_t parameters;
    memcpy(&parameters, &shared_memory->Parameters, sizeof(parameters));
    uint32_t head_checksum = shared_memory->HeadChecksum;

    //　パラメータのフレーム番号が変わっていなければ変更なしと判断する
    if (parameters.FrameNumber == _Parameters.FrameNumber){
        return false;
    }

    // パラメータの先頭と末尾のチェックサム、パラメータ自体から計算したチェックサムを比較し、すべてが等しくなければエラーと判断する
    if ((head_checksum != tail_checksum) || (head_checksum != parameters.CalculateChecksum())){
        return false;
    }

    // パラメータをローカルメモリーにコピーする
    memcpy(&_Parameters, &parameters, sizeof(_Parameters));
    return true;
}

void SharedMemory::ClearParameters(void){
    // ローカルメモリーのパラメータをクリアする
    memset(&_Parameters, 0, sizeof(_Parameters));

    // 共有メモリーをクリアする
    SharedMemory_t *shared_memory = GetNonCachedSharedMemory();
    shared_memory->HeadChecksum = 0;
    shared_memory->Parameters.FrameNumber = 0;
    shared_memory->TailChecksum = 0;
}

SharedMemory_t SharedMemory::_SharedMemory __attribute__((section(".shared")));
SharedMemory_t::Parameters_t SharedMemory::_Parameters;
