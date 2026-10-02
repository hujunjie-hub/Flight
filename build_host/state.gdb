set pagination off
set confirm off
file D:/STM32Project/Flight/cmake-build-debug/Flight.elf
target remote :3333
interrupt

echo \n===== compare flash vs elf =====\n
compare-sections

echo \n===== g_run (engine time anchor) =====\n
p g_run.now_gpst
p g_run.gnss_time
p g_run.step_us_avg
p g_run.step_us_max
p g_run.imu_cnt
p g_run.gnss_cnt
p g_run.mag_cnt
p g_run.mag_seq
p s_aux_mag.seq
p g_run.cov_warn
p kf_math::s_kf_stats

echo \n===== link on/off flags (did FinSH process commands?) =====\n
p gins_fused_data_ctx.on
p magout_ctx.on
p gnssout_ctx.on
p barout_ctx.on

echo \n===== thread list =====\n
set $c = &rt_object_container[0]
set $node = $c->object_list.next
set $i = 0
while $node != &$c->object_list && $i < 40
    set $t = (struct rt_thread *)((char *)$node - (char *)&((struct rt_thread *)0)->parent.list)
    printf "%-14s stat=0x%02x prio=%2d sp=%p/%p err=%d\n", $t->name, $t->stat, $t->current_priority, $t->sp, $t->stack_addr, $t->error
    set $node = $node->next
    set $i = $i + 1
end

echo \n===== scheduler =====\n
p rt_thread_ready_priority_group
p rt_current_priority

echo \n===== gnss_time UTC base =====\n
info variables gnss_time
monitor resume
detach
quit
