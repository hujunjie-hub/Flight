import io, sys

edits = []
def patch(path, old, new, count=1):
    edits.append((path, old, new, count))

# mag_calib: bias 也是 float, 一并提升
patch('middleware/calibration/mag_calib.c',
      '    /* 先整体升 double 再运算, 避免混合表达式逐次隐式提升 (m 为 float 阵) */\n    const double r0 = raw_ut[0], r1 = raw_ut[1], r2 = raw_ut[2];',
      '    /* 先整体升 double 再运算, 避免混合表达式逐次隐式提升 (m/bias 均为 float) */\n    const double r0 = raw_ut[0], r1 = raw_ut[1], r2 = raw_ut[2];\n    const double b0 = bias[0], b1 = bias[1], b2 = bias[2];')
patch('middleware/calibration/mag_calib.c',
      '        out_ut[i] = m0 * (r0 - bias[0])\n                  + m1 * (r1 - bias[1])\n                  + m2 * (r2 - bias[2]);',
      '        out_ut[i] = m0 * (r0 - b0)\n                  + m1 * (r1 - b1)\n                  + m2 * (r2 - b2);')

# sensor_cmd: %5d 配 float 实参属 printf UB (union mv/ma 为 float), 转整型打印
patch('rt-thread/components/drivers/sensor/v1/sensor_cmd.c',
      'LOG_I("num:%3d, voltage:%5d mV, timestamp:%5d", num, sensor_data->data.mv, sensor_data->timestamp);',
      'LOG_I("num:%3d, voltage:%5d mV, timestamp:%5d", num, (int)sensor_data->data.mv, sensor_data->timestamp);')
patch('rt-thread/components/drivers/sensor/v1/sensor_cmd.c',
      'LOG_I("num:%3d, current:%5d mA, timestamp:%5d", num, sensor_data->data.ma, sensor_data->timestamp);',
      'LOG_I("num:%3d, current:%5d mA, timestamp:%5d", num, (int)sensor_data->data.ma, sensor_data->timestamp);')
patch('rt-thread/components/drivers/sensor/v1/sensor_cmd.c',
      'LOG_I("num:%3d, power:%5d mW, timestamp:%5d", num, sensor_data->data.mv, sensor_data->timestamp);',
      'LOG_I("num:%3d, power:%5d mW, timestamp:%5d", num, (int)sensor_data->data.mv, sensor_data->timestamp);')

# thread.c: 嵌套块局部 mutex 遮蔽外层局部 -> 改名 pend_mutex
patch('rt-thread/src/thread.c',
      '        struct rt_mutex *mutex = (struct rt_mutex*)thread->pending_object;\n        rt_mutex_drop_thread(mutex, thread);',
      '        struct rt_mutex *pend_mutex = (struct rt_mutex*)thread->pending_object;   /* 避免遮蔽外层局部 mutex */\n        rt_mutex_drop_thread(pend_mutex, thread);')

# thread.c: 参数 entry 遮蔽 TU 内可见的全局 entry -> 改名 entry_fn (仅函数体内部名, 不影响 ABI/头文件)
patch('rt-thread/src/thread.c',
      'static rt_err_t _thread_init(struct rt_thread *thread,\n                             const char       *name,\n                             void (*entry)(void *parameter),',
      'static rt_err_t _thread_init(struct rt_thread *thread,\n                             const char       *name,\n                             void (*entry_fn)(void *parameter),')
patch('rt-thread/src/thread.c',
      '    thread->entry = (void *)entry;',
      '    thread->entry = (void *)entry_fn;')
patch('rt-thread/src/thread.c',
      'rt_err_t rt_thread_init(struct rt_thread *thread,\n                        const char       *name,\n                        void (*entry)(void *parameter),',
      'rt_err_t rt_thread_init(struct rt_thread *thread,\n                        const char       *name,\n                        void (*entry_fn)(void *parameter),')
patch('rt-thread/src/thread.c',
      '                        entry,',
      '                        entry_fn,')
patch('rt-thread/src/thread.c',
      'rt_thread_t rt_thread_create(const char *name,\n                             void (*entry)(void *parameter),',
      'rt_thread_t rt_thread_create(const char *name,\n                             void (*entry_fn)(void *parameter),')
patch('rt-thread/src/thread.c',
      '                 entry,',
      '                 entry_fn,')

files = {}
ok = fail = 0
for path, old, new, cnt in edits:
    if path not in files:
        files[path] = io.open(path, encoding='utf-8').read()
    s = files[path]
    n = s.count(old)
    if n == 0:
        print(f'FAIL (not found): {path}: {old[:60]!r}'); fail += 1; continue
    if cnt == 1 and n > 1:
        print(f'FAIL (ambiguous {n}): {path}: {old[:60]!r}'); fail += 1; continue
    files[path] = s.replace(old, new)
    ok += 1
for path, s in files.items():
    io.open(path, 'w', encoding='utf-8', newline='').write(s)
print(f'applied {ok}, failed {fail}')
sys.exit(1 if fail else 0)
