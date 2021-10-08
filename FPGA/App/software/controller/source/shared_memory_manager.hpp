/**
 * @file shared_memory_manager.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include <shared_memory.hpp>

/**
 * 共有メモリーへのアクセスする
 */
class SharedMemoryManager {
public:
    /**
     * 初期化する
     */
    static void initialize(void) {
        clearParameters();
    }

    /**
     * エラーフラグのクリアを要求されているか取得する
     * @return エラーフラグのクリアが要求されていればtrueを返す
     */
    static bool isRequestedClearingErrorFlags() {
        return (static_cast<uint32_t>(__builtin_ldwio(&_shared_memory.error_flags)) == 0xFFFFFFFFUL);
    }

    /**
     * エラーフラグを書き込む
     * @param new_error_flags 書き込むフォルトフラグ
     */
    static void writeErrorFlags(uint32_t new_error_flags) {
        __builtin_stwio(&_shared_memory.error_flags, new_error_flags);
    }

    /**
     * フォルトフラグを書き込む
     * @param new_fault_flags 書き込むフォルトフラグ
     */
    static void writeFaultFlags(uint32_t new_fault_flags) {
        __builtin_stwio(&_shared_memory.fault_flags, new_fault_flags);
    }

    /**
     * Parametersの値が有効ならローカルメモリーにコピーする
     * @return データが更新されていればtrueを返す
     */
    static bool updateParameters(void);

    /**
     * Parametersを取得する
     * @return Parametersへの参照
     */
    static const SharedMemory::Parameters& getParameters(void) {
        return _parameters;
    }

    /**
     * Parametersを初期化する
     */
    static void clearParameters(void);

private:
    /**
     * 共有メモリーへのポインタを取得する
     * データキャッシュが存在する場合に備えて非キャッシュ領域のアドレスを使用する
     * @return 共有メモリーへのポインタ
     */
    static SharedMemory* getNonCachedSharedMemory(void) {
        return reinterpret_cast<SharedMemory*>(reinterpret_cast<uint32_t>(&_shared_memory) | 0x80000000UL);
    }

    /// sharedセクションに存在する共有メモリーのデータの実体
    static SharedMemory _shared_memory;

    /// _shared_memoryの中のParametersのコピー
    static SharedMemory::Parameters _parameters;
};
