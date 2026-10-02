set pagination off
set confirm off
target remote :3333
echo \n=== RT TICK ALIVE CHECK (2 reads, 1s apart) ===\n
p/x rt_tick
monitor sleep 1000
p/x rt_tick
echo \n=== USART1 CR1/ISR/BRR ===\n
x/1wx 0x40011000
x/1wx 0x4001101C
x/1wx 0x4001100C
echo \n=== SCHEDULER ===\n
p/x rt_thread_ready_priority_group
p/x rt_thread_current_thread
detach
quit
