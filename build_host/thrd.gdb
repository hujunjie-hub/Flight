set pagination off
set confirm off
target remote :3333
set $t = (struct rt_thread *)ginsout_ctx.thread
p/x $t
p $t->stat
p/x $t->sp
set $sp = $t->sp
echo SAVED_PC:
x/1wx $sp+24
echo SAVED_LR:
x/1wx $sp+20
monitor resume
detach
quit
