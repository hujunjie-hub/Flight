set pagination off
set confirm off
file D:/STM32Project/Flight/cmake-build-debug/Flight.elf
target remote :3333
interrupt

echo \n===== finsh shell internals =====\n
info variables finsh_thread
p finsh_thread
p shell
p shell->device
p shell->device->open_flag
p shell->device->rx_indicate

echo \n===== thread list via priority table =====\n
set $prio = 0
while $prio < 32
    set $node = rt_thread_priority_table[$prio].next
    while $node != &rt_thread_priority_table[$prio]
        set $t = (struct rt_thread *)((char *)$node - (char *)&((struct rt_thread *)0)->tlist)
        printf "prio=%2d %-16s stat=0x%02x\n", $prio, $t->name, $t->stat
        set $node = $node->next
    end
    set $prio = $prio + 1
end

echo \n===== g_run state =====\n
p g_run.now_gpst
p g_run.imu_cnt
p g_run.gnss_cnt
p g_run.mag_cnt
p g_run.step_us_avg
p g_run.step_us_max

monitor resume
detach
quit
