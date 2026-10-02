# 在断言死循环处抓调用栈: 连上后不停板, 直接装断点 continue 等命中
# (板上断言循环每几秒复现一次, 无需复位)
file Flight.elf
target extended-remote 127.0.0.1:3333
set confirm off
set pagination off
break rt_assert_handler
continue
echo \n===ASSERT HIT===\n
bt 12
echo \n===THREADS===\n
info threads
echo \n===STOP THE LOOP===\n
delete breakpoints
interrupt
kill
detach
quit
