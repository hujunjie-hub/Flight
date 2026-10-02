set pagination off
set confirm off
file D:/STM32Project/Flight/cmake-build-debug/Flight.elf
target remote :3333
interrupt

echo \n===== nav state =====\n
p s_engine->pvacur_.pos
p s_engine->pvacur_.vel
p s_engine->pvacur_.att.euler
echo skip
echo skip

echo \n===== kf_math internal P diagonal (idx: value) =====\n
set $i = 0
while $i < 21
    printf "P[%2d] = %12.6g\n", $i, 'kf_math.cpp'::s_dtc.P[$i*21+$i]
    set $i = $i + 1
end

echo \n===== P off-diag extremes =====\n
set $max = 0.0
set $i = 0
while $i < 21
    set $j = 0
    while $j < 21
        if $i != $j
            set $v = 'kf_math.cpp'::s_dtc.P[$i*21+$j]
            if ($v > $max || $v < -$max)
                set $max = ($v > 0) ? $v : -$v
            end
        end
        set $j = $j + 1
    end
    set $i = $i + 1
end
printf "max |P offdiag| = %g\n", $max

echo \n===== Phi row max =====\n
set $i = 0
set $max = 0.0
while $i < 21
    set $j = 0
    while $j < 21
        set $v = 'kf_math.cpp'::s_dtc.Phi[$i*21+$j]
        if $i != $j && (($v > $max) || ($v < -$max))
            set $max = ($v > 0) ? $v : -$v
        end
        set $j = $j + 1
    end
    set $i = $i + 1
end
printf "max |Phi offdiag| = %g\n", $max

echo \n===== kf stats =====\n
p 'kf_math.cpp'::s_pd_avg
p 'kf_math.cpp'::s_pd_max
p 'kf_math.cpp'::s_up_avg
p 'kf_math.cpp'::s_up_max

echo \n===== thread list =====\n
set $c = &rt_object_container[0]
set $node = $c->object_list.next
set $i = 0
while $node != &$c->object_list && $i < 40
    set $t = (struct rt_thread *)((char *)$node - (char *)&((struct rt_thread *)0)->parent.list)
    printf "%-16s stat=0x%02x prio=%2d err=%d\n", $t->name, $t->stat, $t->current_priority, $t->error
    set $node = $node->next
    set $i = $i + 1
end

echo \n===== mag/baro data link status =====\n
echo skip
echo skip
monitor resume
detach
quit
