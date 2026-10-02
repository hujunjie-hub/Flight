set pagination off
set confirm off
target remote :3333
p magout_ctx.on
p gnssout_ctx.on
monitor resume
detach
quit
