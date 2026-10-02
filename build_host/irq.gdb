set pagination off
set confirm off
target remote :3333
echo \n=== IRQ MASK STATE ===\n
p/x $primask
p/x $faultmask
p/x $basepri
echo \n=== SYSTICK CTRL/LOAD/VAL ===\n
x/1wx 0xE000E010
x/1wx 0xE000E014
x/1wx 0xE000E018
echo \n=== NVIC ISER0-2 ===\n
x/3wx 0xE000E100
detach
quit
