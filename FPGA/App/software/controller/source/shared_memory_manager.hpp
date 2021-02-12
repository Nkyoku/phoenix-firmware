#pragma once

#include <stdint.h>
#include <string.h>
#include <shared_memory.hpp>

/**
 * 共有メモリーへのアクセスする
 */
class SharedMemory {
public:
    /**
     * 初期化する
     */
    static void Initialize(void){
        ClearParameters();
    }
    
    /**
     * エラーフラグのクリアを要求されているか取得する
     * @return エラーフラグのクリアが要求されていればtrueを返す
     */
    static bool IsRequestedClearingErrorFlags(){
        return (static_cast<uint32_t>(__builtin_ldwio(&_SharedMemory.ErrorFlags)) == 0xFFFFFFFFUL);
    }

    /**
     * エラーフラグを書き込む
     * @param new_error_flags 書き込むフォルトフラグ
     */
    static void WriteErrorFlags(uint32_t new_error_flags){
        __builtin_stwio(&_SharedMemory.ErrorFlags, new_error_flags);
    }

    /**
     * フォルトフラグを書き込む
     * @param new_fault_flags 書き込むフォルトフラグ
     */
    static void WriteFaultFlags(uint32_t new_fault_flags){
        __builtin_stwio(&_SharedMemory.FaultFlags, new_fault_flags);
    }

    /**
     * Parametersの値が有効ならローカルメモリーにコピーする
     * @return データが更新されていればtrueを返す
     */
    static bool UpdateParameters(void);

    /**
     * Parametersを取得する
     * @return Parametersへの参照
     */
    static const SharedMemory_t::Parameters_t& GetParameters(void){
        return _Parameters;
    }

    /**
     * Parametersを初期化する
     */
    static void ClearParameters(void);

private:
    /**
     * 共有メモリーへのポインタを取得する
     * データキャッシュが存在する場合に備えて非キャッシュ領域のアドレスを使用する
     * @return 共有メモリーへのポインタ
     */
    static SharedMemory_t* GetNonCachedSharedMemory(void){
        return reinterpret_cast<SharedMemory_t*>(reinterpret_cast<uint32_t>(&_SharedMemory) | 0x80000000UL);
    }

    /// sharedセクションに存在する共有メモリーのデータの実体
    static SharedMemory_t _SharedMemory;

    /// _SharedMemoryの中のParametersのコピー
    static SharedMemory_t::Parameters_t _Parameters;
};
