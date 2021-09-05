#include "shared_memory_manager.hpp"

bool SharedMemoryManager::updateParameters(void){
    // 共有メモリーからパラメータを一時的にコピーする
    SharedMemory *shared_memory = getNonCachedSharedMemory();
    uint32_t tail_checksum = shared_memory->tail_checksum;
    SharedMemory::Parameters parameters;
    memcpy(&parameters, &shared_memory->parameters, sizeof(parameters));
    uint32_t head_checksum = shared_memory->head_checksum;

    //　パラメータのフレーム番号が変わっていなければ変更なしと判断する
    if (parameters.frame_number == _parameters.frame_number){
        return false;
    }

    // パラメータの先頭と末尾のチェックサム、パラメータ自体から計算したチェックサムを比較し、すべてが等しくなければエラーと判断する
    if ((head_checksum != tail_checksum) || (head_checksum != parameters.calculateChecksum())){
        return false;
    }

    // パラメータをローカルメモリーにコピーする
    memcpy(&_parameters, &parameters, sizeof(_parameters));
    return true;
}

void SharedMemoryManager::clearParameters(void){
    // ローカルメモリーのパラメータをクリアする
    memset(&_parameters, 0, sizeof(_parameters));

    // 共有メモリーをクリアする
    SharedMemory *shared_memory = getNonCachedSharedMemory();
    shared_memory->head_checksum = 0;
    shared_memory->parameters.frame_number = 0;
    shared_memory->tail_checksum = 0;
}

SharedMemory SharedMemoryManager::_shared_memory __attribute__((section(".shared")));
SharedMemory::Parameters SharedMemoryManager::_parameters;
