#include "stm32f4xx.h"   // F446はCortex-M4
#include <unistd.h>
#include <sys/errno.h>

static inline void itm_write_blocking(char ch)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U) return; // ITM未有効なら捨てる
    if ((ITM->TER & 1U) == 0U) return;                 // Stimulus port 0のみ
    while (ITM->PORT[0].u32 == 0U) { __NOP(); }
    ITM_SendChar((uint32_t)ch);
}

int _write(int file, char *ptr, int len)
{
    if (file == STDOUT_FILENO || file == STDERR_FILENO) {
        for (int i = 0; i < len; i++) itm_write_blocking(ptr[i]);
        return len;
    }
    errno = EBADF;
    return -1;
}
