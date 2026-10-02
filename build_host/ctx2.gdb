set pagination off
set confirm off
target remote :3333
p ginsout_ctx.thread
p ginsout_ctx.on
p ginsout_ctx.lines
p ginsout_ctx.uart
monitor resume
detach
quit
