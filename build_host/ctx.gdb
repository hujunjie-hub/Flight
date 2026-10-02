set pagination off
set confirm off
target remote :3333
echo GINSOUT_CTX:
p ginsout_ctx
echo GNSSOUT_ON:
p gnssout_ctx.on
echo MAGOUT_ON:
p magout_ctx.on
monitor resume
detach
quit
