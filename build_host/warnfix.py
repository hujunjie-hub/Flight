import io, sys

BS = chr(92)
edits = []

def patch(path, old, new, count=1):
    edits.append((path, old, new, count))

# ---------- 我方代码: 混合运算 float->double ----------
patch('middleware/KF-GINS/src/common/angle.h',
      '    static float rad2deg(float rad) {\n        return rad * R2D;\n    }\n\n    static float deg2rad(float deg) {\n        return deg * D2R;\n    }',
      '    static float rad2deg(float rad) {\n        return rad * (float)R2D;   /* float 重载内用 float 常数, 避免逐次 double 提升 */\n    }\n\n    static float deg2rad(float deg) {\n        return deg * (float)D2R;\n    }')

patch('middleware/calibration/mag_calib.c',
      '    for (int i = 0; i < 3; i++)\n    {\n        double v = raw_ut[0] - bias[0];\n\n        out_ut[i] = m[i][0] * v\n                  + m[i][1] * (raw_ut[1] - bias[1])\n                  + m[i][2] * (raw_ut[2] - bias[2]);\n    }',
      '    /* 先整体升 double 再运算, 避免六个混合表达式逐次隐式提升 */\n    const double r0 = raw_ut[0], r1 = raw_ut[1], r2 = raw_ut[2];\n\n    for (int i = 0; i < 3; i++)\n    {\n        out_ut[i] = m[i][0] * (r0 - bias[0])\n                  + m[i][1] * (r1 - bias[1])\n                  + m[i][2] * (r2 - bias[2]);\n    }')

# gins_bridge: 需要逐段看原文, 用宽松定位
patch('middleware/gins/gins_bridge.cpp',
      'gs->vn > -5e2 && gs->vn < 5e2 &&\n           gs->ve > -5e2 && gs->ve < 5e2;',
      '(double)gs->vn > -5e2 && (double)gs->vn < 5e2 &&\n           (double)gs->ve > -5e2 && (double)gs->ve < 5e2;')

patch('middleware/gins/gins_bridge.cpp',
      'vh = sqrt((double)gs->vn * gs->vn + (double)gs->ve * gs->ve);',
      'vh = sqrt((double)gs->vn * (double)gs->vn + (double)gs->ve * (double)gs->ve);')

patch('middleware/gins/gins_bridge.cpp',
      'imu0.dtheta << s0->gyro[0] * dt, s0->gyro[1] * dt, s0->gyro[2] * dt;',
      'imu0.dtheta << (double)s0->gyro[0] * dt, (double)s0->gyro[1] * dt, (double)s0->gyro[2] * dt;')

patch('middleware/gins/gins_bridge.cpp',
      'double mnorm = sqrt((double)m.mag_ut[0] * m.mag_ut[0] +\n                                (double)m.mag_ut[1] * m.mag_ut[1] +\n                                (double)m.mag_ut[2] * m.mag_ut[2]);',
      'double mnorm = sqrt((double)m.mag_ut[0] * (double)m.mag_ut[0] +\n                                (double)m.mag_ut[1] * (double)m.mag_ut[1] +\n                                (double)m.mag_ut[2] * (double)m.mag_ut[2]);')

# ---------- 我方代码: 变参日志显式 (double) ----------
patch('middleware/gins/gins_bridge.cpp',
      'gs.fix_type, gs.satellites, gs.hdop,',
      'gs.fix_type, gs.satellites, (double)gs.hdop,')

patch('middleware/gins/gins_bridge.cpp',
      'g_run.step_us_avg / 1000.0f, g_run.step_us_max / 1000.0f,',
      '(double)(g_run.step_us_avg / 1000.0f), (double)(g_run.step_us_max / 1000.0f),')

patch('middleware/data/gnss_data.c',
      'ctx.last.latitude_deg, ctx.last.longitude_deg,\n              ctx.last.altitude_m,\n              ctx.last.vn, ctx.last.ve, ctx.last.vu);',
      '(double)ctx.last.latitude_deg, (double)ctx.last.longitude_deg,\n              (double)ctx.last.altitude_m,\n              (double)ctx.last.vn, (double)ctx.last.ve, (double)ctx.last.vu);')

patch('middleware/protocol/nmea/um982_nmea.c',
      'LOG_I("position: lat=%.7f deg lon=%.7f deg",\n          d.position.latitude, d.position.longitude);',
      'LOG_I("position: lat=%.7f deg lon=%.7f deg",\n          (double)d.position.latitude, (double)d.position.longitude);')

patch('middleware/protocol/nmea/um982_nmea.c',
      'LOG_I("          alt=%.1f m (ellipsoid, geoid=%.1f) %s",\n          d.position.altitude, d.position.geoid_sep,',
      'LOG_I("          alt=%.1f m (ellipsoid, geoid=%.1f) %s",\n          (double)d.position.altitude, (double)d.position.geoid_sep,')

patch('middleware/protocol/nmea/um982_nmea.c',
      'LOG_I("velocity: vn=%.3f ve=%.3f vu=%.3f m/s valid=%d",\n          d.velocity.vn, d.velocity.ve, d.velocity.vu, d.vel_valid);',
      'LOG_I("velocity: vn=%.3f ve=%.3f vu=%.3f m/s valid=%d",\n          (double)d.velocity.vn, (double)d.velocity.ve, (double)d.velocity.vu, d.vel_valid);')

patch('middleware/protocol/nmea/um982_nmea.c',
      'd.status.fix_type, d.status.rtk_status, d.status.satellites,\n          d.status.hdop);',
      'd.status.fix_type, d.status.rtk_status, d.status.satellites,\n          (double)d.status.hdop);')

patch('middleware/sensor/sensor_bmp585.c',
      'LOG_W("BMP585 pressure implausible (%.1f hPa) x%d, reconfiguring",\n                      pa / 100.0f, BMP585_FETCH_BAD_RECFG_N);',
      'LOG_W("BMP585 pressure implausible (%.1f hPa) x%d, reconfiguring",\n                      (double)(pa / 100.0f), BMP585_FETCH_BAD_RECFG_N);')

# ---------- 上游 RT-Thread 机械修复 ----------
patch('rt-thread/components/drivers/serial/dev_serial_v2.c',
      'rt_uint8_t *put_ptr;\n                /* Get the linear length buffer from ringbuffer */',
      'rt_uint8_t *put_ptr = RT_NULL;   /* get_linear_buffer 空环时不写, 防未初始化读 */\n                /* Get the linear length buffer from ringbuffer */')

patch('rt-thread/components/drivers/serial/dev_serial_v2.c',
      'if (rb->buffer_size - rb->read_index > size)',
      'if ((rt_size_t)(rb->buffer_size - rb->read_index) > size)')

patch('rt-thread/components/drivers/serial/dev_serial_v2.c',
      'if (delta_tick >= base_rx_timeout)',
      'if (delta_tick >= (rt_tick_t)base_rx_timeout)')

patch('rt-thread/components/drivers/serial/dev_serial_v2.c',
      'if (delta_tick >= base_tx_timeout)',
      'if (delta_tick >= (rt_tick_t)base_tx_timeout)')

patch('rt-thread/components/drivers/ipc/ringbuffer.c',
      'if (rb->buffer_size - rb->write_index > length)',
      'if ((rt_size_t)(rb->buffer_size - rb->write_index) > length)', count=2)

patch('rt-thread/components/drivers/ipc/ringbuffer.c',
      'if (rb->buffer_size - rb->read_index > length)',
      'if ((rt_size_t)(rb->buffer_size - rb->read_index) > length)')

patch('rt-thread/components/drivers/ipc/ringbuffer.c',
      'if (length > rb->buffer_size)',
      'if (length > (rt_size_t)rb->buffer_size)')

patch('rt-thread/components/drivers/ipc/condvar.c',
      'if (acq_mtx_succ == 1 || waiting_mtx == (size_t)mtx)',
      'if (acq_mtx_succ == 1 || (size_t)waiting_mtx == (size_t)mtx)')

patch('rt-thread/src/klibc/kerrno.c',
      'for (i = 0; i < sizeof(rt_errno_strs) / sizeof(rt_errno_strs[0]); i++)',
      'for (i = 0; i < (int)(sizeof(rt_errno_strs) / sizeof(rt_errno_strs[0])); i++)')

patch('rt-thread/src/kservice.c',
      'for (rt_size_t i = 0; i < buflen && buffer[i] != 0; i++)',
      'for (rt_size_t i = 0; i < (rt_size_t)buflen && buffer[i] != 0; i++)')

patch('rt-thread/src/klibc/rt_vsscanf.c',
      'if (width > *inr) {',
      'if ((size_t)width > (size_t)*inr) {')

patch('rt-thread/src/klibc/rt_vsscanf.c',
      'if (((flags & POINTER) != 0) && ((*inr) >= FORMAT_NIL_STR_LEN)',
      'if (((flags & POINTER) != 0) && ((size_t)(*inr) >= (size_t)FORMAT_NIL_STR_LEN)')

patch('rt-thread/src/klibc/rt_vsscanf.c',
      'if ((width != 0) && (width < *inr)) {',
      'if ((width != 0) && ((size_t)width < (size_t)*inr)) {')

patch('rt-thread/components/libc/compilers/common/cwchar.c',
      'if (ucs < table[0].first || ucs > table[max].last)',
      'if ((long)ucs < table[0].first || (long)ucs > table[max].last)')

patch('rt-thread/components/libc/compilers/common/cwchar.c',
      'if (ucs > table[mid].last)',
      'if ((long)ucs > table[mid].last)')

patch('rt-thread/components/libc/compilers/common/cwchar.c',
      'else if (ucs < table[mid].first)',
      'else if ((long)ucs < table[mid].first)')

patch('libraries/HAL_Drivers/drivers/drv_hard_i2c.c',
      'for (int i = 0; i < obj_num; i++)',
      'for (int i = 0; i < (int)obj_num; i++)')

patch('rt-thread/components/libc/cplusplus/cxx_crt.cpp',
      'void operator delete(void *ptr)\n{\n    rt_free(ptr);\n}\n\nvoid operator delete[](void *ptr)\n{\n    return rt_free(ptr);\n}',
      'void operator delete(void *ptr)\n{\n    rt_free(ptr);\n}\n\n/* C++14 带尺寸释放: 与上面等价 (rt_free 无尺寸变体), 消除 -Wsized-deallocation */\nvoid operator delete(void *ptr, size_t size)\n{\n    (void)size;\n    rt_free(ptr);\n}\n\nvoid operator delete[](void *ptr)\n{\n    return rt_free(ptr);\n}\n\nvoid operator delete[](void *ptr, size_t size)\n{\n    (void)size;\n    rt_free(ptr);\n}')

patch('middleware/KF-GINS/ThirdParty/eigen-3.3.9/Eigen/src/Core/util/Memory.h',
      '    std::size_t huge = static_cast<std::size_t>(-1);\n    ::operator new(huge);',
      '    std::size_t huge = static_cast<std::size_t>(-1);\n    (void)::operator new(huge);   /* Flight: 显式弃置返回值, 消除 nodiscard 告警 (Eigen 3.4 已同改) */')

# ---------- dev_i2c_core ?: 符号统一 ----------
patch('rt-thread/components/drivers/i2c/dev_i2c_core.c',
      'return (ret == 1) ? count : ret;',
      'return (ret == 1) ? (rt_ssize_t)count : ret;', count=2)

# ---------- 执行 ----------
files = {}
ok = fail = 0
for path, old, new, cnt in edits:
    if path not in files:
        files[path] = io.open(path, encoding='utf-8').read()
    s = files[path]
    n = s.count(old)
    if n == 0:
        print(f'FAIL (not found): {path}: {old[:60]!r}')
        fail += 1
        continue
    if cnt == 1 and n > 1:
        print(f'FAIL (ambiguous {n}): {path}: {old[:60]!r}')
        fail += 1
        continue
    files[path] = s.replace(old, new)
    ok += 1

for path, s in files.items():
    io.open(path, 'w', encoding='utf-8', newline='').write(s)

print(f'applied {ok}, failed {fail}')
sys.exit(1 if fail else 0)
