/**
 * @file critical_section.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <sys/alt_irq.h>

/**
 * クリティカルセクションを作り出す
 */
class CriticalSection {
public:
    CriticalSection(void) {
        _context = alt_irq_disable_all();
        __builtin_sync();
    }
    
    ~CriticalSection() {
        alt_irq_enable_all(_context);
    }

private:
    CriticalSection(const CriticalSection&);

    alt_irq_context _context;    
};
