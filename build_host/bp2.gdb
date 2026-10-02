set pagination off
set confirm off
target remote :3333
break ginsout_link_init
monitor reset run
continue
next
p ginsout_ctx.uart
next
p ginsout_ctx.uart
next
p ginsout_ctx.thread
monitor resume
detach
quit
