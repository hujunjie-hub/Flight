set pagination off
set confirm off
target remote :3333
echo BASEPRI=
p/x $basepri
echo SYST_VAL_1=
x/1wx 0xE000E018
monitor sleep 200
echo SYST_VAL_2=
x/1wx 0xE000E018
echo ICSR=
x/1wx 0xE000ED04
echo SHPR3=
x/1wx 0xE000ED20
detach
quit
