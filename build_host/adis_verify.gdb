target remote :3333
set pagination off
printf "=== ADIS16505 chain state ===\n"
printf "prod_id=%d (16505=ok)  dr_mode=%d  chain_on=%d  dma_busy=%d\n", adis_dev.prod_id, adis_dev.dr_mode, adis_dev.chain_on, adis_dev.dma_busy
printf "dec_rate=%d gyro_idx=%d sample_valid=%d\n", adis_dev.dec_rate, adis_dev.gyro_idx, adis_dev.sample_valid
printf "sample: cntr=%d diag=0x%x gyro=[%d %d %d] acce=[%d %d %d] temp=%d\n", adis_dev.sample.data_cntr, adis_dev.sample.diag_stat, adis_dev.sample.gyro[0], adis_dev.sample.gyro[1], adis_dev.sample.gyro[2], adis_dev.sample.acce[0], adis_dev.sample.acce[1], adis_dev.sample.acce[2], adis_dev.sample.temp
printf "stats: miss=%u drop=%u chk_err=%u diag_err=%u dma_err=%u overrun=%u recover=%u\n", adis_dev.stats.miss, adis_dev.stats.drop, adis_dev.stats.chk_err, adis_dev.stats.diag_err, adis_dev.stats.dma_err, adis_dev.stats.overrun, adis_dev.stats.recover
printf "queue: head=%d tail=%d  rt_tick=%u\n", adis_dev.q_head, adis_dev.q_tail, rt_tick
monitor resume
detach
quit
