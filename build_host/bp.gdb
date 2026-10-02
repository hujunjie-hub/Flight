set pagination off
set confirm off
target remote :3333
break ginsout_link_init
monitor reset run
continue
echo HIT_GINSOUT
finish
echo RET:
p ginsout_ctx.uart
p ginsout_ctx.thread
monitor resume
detach
quit
