#include <stdio.h>
#ifdef __ASSEMBLER__
#define __ASM_STR(x)	x
#else
#define __ASM_STR(x)	#x
#endif
#define CSR_CYCLE			0xc00
#define csr_read(csr)                                           \
	({                                                      \
		register unsigned long __v;                     \
		__asm__ __volatile__("csrr %0, " __ASM_STR(csr) \
				     : "=r"(__v)                \
				     :                          \
				     : "memory");               \
		__v;                                            \
	})


int main(void){
    unsigned long cycle;
    cycle = csr_read(CSR_CYCLE);
    printf("Cycle count: %lu\n", cycle);
}