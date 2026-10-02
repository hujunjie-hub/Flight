# -*- coding: utf-8 -*-
"""SWD 直读开发板运行状态 (无串口环境下的 KF-GINS 诊断)."""
import subprocess, re, struct, sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

def read_words(addr, nbytes):
    n = (nbytes + 3) // 4
    p = subprocess.run([CLI, "-c", "port=SWD mode=HotPlug",
                        "-r32", hex(addr), str(nbytes)],
                       capture_output=True, text=True, timeout=30)
    words = []
    for line in p.stdout.splitlines():
        m = re.match(r"^0x[0-9A-Fa-f]{8}\s*:\s*(.*)$", line)
        if m:
            words += [int(w, 16) for w in m.group(1).split()]
    if len(words) < n:
        raise RuntimeError(f"read {addr:#x}: got {len(words)}/{n} words\n{p.stdout[-500:]}")
    return struct.pack("<%dI" % n, *words)

def u32(b, o): return struct.unpack_from("<I", b, o)[0]
def u64(b, o): return struct.unpack_from("<Q", b, o)[0]
def f32(b, o): return struct.unpack_from("<f", b, o)[0]
def f64(b, o): return struct.unpack_from("<d", b, o)[0]
def i32(b, o): return struct.unpack_from("<i", b, o)[0]

# ---- nav @ 0x2400a9c0 (um982_nmea) ----
nav = read_words(0x2400a9c0, 0x68)
print("=== UM982 NMEA 解析状态 (nav) ===")
print(f"  utc_sec={u32(nav,0)} utc_usec={u32(nav,4)} time_valid={i32(nav,56)}")
print(f"  lat={f64(nav,8):.7f} lon={f64(nav,16):.7f} alt={f32(nav,24):.2f} geoid={f32(nav,28):.2f}")
print(f"  vn={f32(nav,32):.3f} ve={f32(nav,36):.3f} vu={f32(nav,40):.3f}")
print(f"  fix_type={nav[44]} sats={nav[45]} hdop={f32(nav,48):.2f} rtk={nav[52]}")
print(f"  pos_valid={i32(nav,60)} vel_valid={i32(nav,64)} update_cnt={u32(nav,68)}")
print(f"  rmc_cnt={u32(nav,80)} gga_cnt={u32(nav,84)} zda_cnt={u32(nav,88)} csum_err={u32(nav,92)} field_err={u32(nav,96)}")

# ---- g_map @ 0x2400ae08 (timebase) ----
gm = read_words(0x2400ae08, 0x80)
print("\n=== PPS 时间同步 (timebase g_map) ===")
print(f"  win[0..2] tmcu_pps={u64(gm,0)} {u64(gm,16)} {u64(gm,32)}")
print(f"  valid_cnt={gm[48]}")
print(f"  map: t_mcu_base={u64(gm,56)} t_utc_base={u64(gm,64)} scale={f64(gm,72):.9f} valid={gm[80]}")
print(f"  last_pair_mcu={u64(gm,88)}")
names = ["pair_ok","rej_no_pps","rej_arrival","rej_interval","rej_slope","rej_residual","restart","erase_repair"]
st = {n: u32(gm, 96+4*i) for i, n in enumerate(names)}
print("  st:", " ".join(f"{k}={v}" for k, v in st.items()))

# ---- s_pps @ 0x2400adf8 ----
sp = read_words(0x2400adf8, 0x10)
print("\n=== PPS 硬件捕获 (s_pps) ===")
print(f"  seq={u32(sp,0)} t_mcu={u64(sp,8)}")

# ---- g_run @ 0x2400bb10 ----
gr = read_words(0x2400bb10, 0x50)
print("\n=== gins 桥接运行态 (g_run) ===")
print(f"  imu_cnt={u32(gr,0)} gnss_cnt={u32(gr,4)} cov_warn={u32(gr,8)} drop={u32(gr,12)} gnss_stale={u32(gr,16)}")
print(f"  mag_cnt={u32(gr,20)} baro_cnt={u32(gr,24)} mag_stale={u32(gr,36)} baro_stale={u32(gr,40)}")
print(f"  gyro_lsb_mdps={f64(gr,48):.3f} now_gpst={f64(gr,56):.3f} gnss_time={f64(gr,64):.3f}")
print(f"  step_us_avg={f32(gr,72):.1f} step_us_max={f32(gr,76):.1f}")

# ---- g_anchor @ 0x2400bb60 ----
ga = read_words(0x2400bb60, 0x18)
print("\n=== 引擎时间锚点 (g_anchor) ===")
print(f"  tmcu={u64(ga,0)} gpst={f64(ga,8):.3f} valid={i32(ga,16)}")

# ---- g_sol @ 0x2400bb78 ----
s = read_words(0x2400bb78, 0xa0)
print("\n=== 解算快照 (g_sol) ===")
print(f"  ready={i32(s,0)} time={f64(s,8):.3f}")
print(f"  lat={f64(s,16):.7f} lon={f64(s,24):.7f} alt={f64(s,32):.2f}")
print(f"  vn={f64(s,40):.3f} ve={f64(s,48):.3f} vd={f64(s,56):.3f}")
print(f"  roll={f64(s,64):.3f} pitch={f64(s,72):.3f} yaw={f64(s,80):.3f}")
print(f"  imu={u32(s,88)} gnss={u32(s,92)} stale={u32(s,96)} age={u32(s,100)}ms covwarn={u32(s,104)} drop={u32(s,108)}")
print(f"  mag={u32(s,120)} magrej={u32(s,124)} magstale={u32(s,128)} baro={u32(s,132)} barorej={u32(s,136)} barostale={u32(s,140)}")
print(f"  baro_height={f64(s,144):.2f} mag_decl={f64(s,152):.1f}")
