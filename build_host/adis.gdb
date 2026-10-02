set pagination off
set confirm off
file D:/STM32Project/Flight/cmake-build-debug/Flight.elf
target remote :3333
interrupt
echo \n===== adis sample #1 =====\n
p 'sensor_adis16505.c'::adis_dev.sample
monitor resume
shell sleep 0.5
interrupt
echo \n===== adis sample #2 (500ms later) =====\n
p 'sensor_adis16505.c'::adis_dev.sample
monitor resume
detach
quit
