set pagination off
set confirm off
target remote :3333
echo \n=== REGS ===\n
info registers pc lr sp xpsr
echo \n=== BACKTRACE ===\n
bt
echo \n=== FAULT CFSR/HFSR/BFAR ===\n
x/1wx 0xE000ED28
x/1wx 0xE000ED2C
x/1wx 0xE000ED38
echo \n=== RESUME TARGET ===\n
monitor resume
detach
quit
