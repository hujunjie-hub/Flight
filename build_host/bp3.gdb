set pagination off
set confirm off
target remote :3333
break main.c:833
break main.c:841
break main.c:853
break main.c:856
monitor reset run
continue
echo BP1_HIT_PC:
p/x $pc
continue
echo BP2_HIT_PC:
p/x $pc
monitor resume
detach
quit
