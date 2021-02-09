#pragma once

#include <sys/alt_irq.h>

/**
 * クリティカルセクションを作り出す
 */
class CriticalSection {
public:
    CriticalSection(void) {
        _Context = alt_irq_disable_all();
        __builtin_sync();
    }
    
    ~CriticalSection() {
        alt_irq_enable_all(_Context);
    }

private:
    CriticalSection(const CriticalSection&);

    alt_irq_context _Context;    
};
