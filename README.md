Coremark for STM32F4R5 @ 120 MHz.
ARMCC 5.06 Optimized for speed.
#define ITERATIONS	5000

With microlib:
CoreMark 1.0 : 90.614183 / ARMCC 5060422 -c --cpu Cortex-M4.fp -g -O3 -Otime --apcs=interwork --split_sections --C99 / STACK

Without microlib:
CoreMark 1.0 : 91.452819 / ARMCC 5060422 -c --cpu Cortex-M4.fp -g -O3 -Otime --apcs=interwork --split_sections --C99 / STACK

=================================================================
With microlib
Coremark For STM32F4R5 Nucleo Board @ 120000000 Hz, 120000000, 120000000, 1000
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 55179
Total time (secs): 55.179000
Iterations/Sec   : 90.614183
Iterations       : 5000
Compiler version : ARMCC 5060422
Compiler flags   : -c --cpu Cortex-M4.fp -g -O3 Otime --apcs=interwork --split_sections --C99
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0xbd59
Correct operation validated. See readme.txt for run and reporting rules.
CoreMark 1.0 : 90.614183 / ARMCC 5060422 -c --cpu Cortex-M4.fp -g -O3 -Otime --apcs=interwork --split_sections --C99 / STACK
End of Coremark

Without microlib
Coremark For STM32F4R5 Nucleo Board @ 120000000 Hz, 120000000, 120000000, 1000
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 54673
Total time (secs): 54.673000
Iterations/Sec   : 91.452819
Iterations       : 5000
Compiler version : ARMCC 5060422
Compiler flags   : -c --cpu Cortex-M4.fp -g -O3 -Otime --apcs=interwork --split_sections --C99
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0xbd59
Correct operation validated. See readme.txt for run and reporting rules.
CoreMark 1.0 : 91.452819 / ARMCC 5060422 -c --cpu Cortex-M4.fp -g -O3 -Otime --apcs=interwork --split_sections --C99 / STACK
End of Coremark

