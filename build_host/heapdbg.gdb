set pagination off
set confirm off
file D:/STM32Project/Flight/cmake-build-debug/Flight.elf
target remote :3333
interrupt

echo \n===== heap (small mem) =====\n
p 'mem.c'::heap_ptr
p 'mem.c'::heap_end
p 'mem.c'::used_mem
p 'mem.c'::max_mem
printf "heap size  = %lu B\n", (unsigned long)('mem.c'::heap_end - 'mem.c'::heap_ptr)
printf "heap used  = %lu B\n", (unsigned long)'mem.c'::used_mem
printf "heap maxed = %lu B\n", (unsigned long)'mem.c'::max_mem

echo \n===== threads =====\n
set $prio = 0
while $prio < 32
    set $node = rt_thread_priority_table[$prio].next
    while $node != &rt_thread_priority_table[$prio]
        set $t = (struct rt_thread *)((char *)$node - (char *)&((struct rt_thread *)0)->list)
        printf "prio=%2d %-16s stat=0x%02x\n", $prio, $t->name, $t->stat
        set $node = $node->next
    end
    set $prio = $prio + 1
end

echo \n===== g_run =====\n
p g_run.now_gpst
p g_run.imu_cnt
p g_run.gnss_cnt
p g_run.mag_cnt
p g_run.step_us_avg
p g_run.step_us_max
p s_engine

monitor resume
detach
quit
