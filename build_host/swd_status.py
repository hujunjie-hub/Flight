# -*- coding: utf-8 -*-
"""综合状态: PPS 配对 + g_sol 解算快照 + g_run + nav 概要."""
import subprocess, re, struct, sys, io

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
CLI = r"D:\STM32CubeCLT\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

def rd(addr, nbytes):
    p = subprocess.run([CLI, "-c", "port=SWD mode=HotPlug",
                        "-r32", hex(addr), str(nbytes)],
                       capture_output=True, text=True, timeout=30)
    words = []
    for line in p.stdout.splitlines():
        m = re.match(r"^0x[0-9A-F]{8}\s*:\s*(.*)$", line)
        if m: words += [int(w, 16) for w in m.group(1).split()]
    n = (nbytes + 3) // 4
    if len(words) < n: raise RuntimeError(f"short read {addr:#x}")
    return struct.pack("<%dI" % n, *words)

def u32(b,o): return struct.unpack_from("<I",b,o)[0]
def f64(b,o): return struct.unpack_from("<d",b,o)[0]
def i32(b,o): return struct.unpack_from("<i",b,o)[0]

gm = rd(0x2400ae08, 128)
print("=== PPS 配对 (g_map) ===")
print(f"滑窗 valid_cnt={gm[48]}  map.valid={gm[80]}  scale={f64(gm,72):.9f}")
names = ["pair_ok","rej_no_pps","rej_arrival","rej_interval","rej_slope","rej_residual","restart","erase_repair"]
st = {n: u32(gm,96+4*i) for i,n in enumerate(names)}
print("st:", " ".join(f"{k}={v}" for k,v in st.items()))

gr = rd(0x2400bb10, 80)
print("\n=== gins 桥接运行态 (g_run) ===")
print(f"imu={u32(gr,0)} gnss={u32(gr,4)} covwarn={u32(gr,8)} drop={u32(gr,12)} gnss_stale={u32(gr,16)}")
print(f"mag={u32(gr,20)} baro={u32(gr,24)} mag_stale={u32(gr,36)} baro_stale={u32(gr,40)}")
print(f"now_gpst={f64(gr,56):.1f} gnss_time={f64(gr,64):.1f} step_us(avg/max)={struct.unpack_from('<ff',gr,72)[0]:.0f}/{struct.unpack_from('<ff',gr,72)[1]:.0f}")

s = rd(0x2400bb78, 160)
print("\n=== 解算快照 (g_sol) ===")
print(f"ready={i32(s,0)} time={f64(s,8):.1f}")
print(f"lat={f64(s,16):.7f} lon={f64(s,24):.7f} alt={f64(s,32):.2f}")
print(f"vn={f64(s,40):.3f} ve={f64(s,48):.3f} vd={f64(s,56):.3f}")
print(f"roll={f64(s,64):.3f} pitch={f64(s,72):.3f} yaw={f64(s,80):.3f}")
print(f"imu={u32(s,88)} gnss={u32(s,92)} stale={u32(s,96)} age={u32(s,100)}ms covwarn={u32(s,104)} drop={u32(s,108)}")
print(f"mag={u32(s,120)} magrej={u32(s,124)} baro={u32(s,132)} barorej={u32(s,136)} baro_h={f64(s,144):.1f} decl={f64(s,152):.1f}")

n = rd(0x2400a9c0, 104)
print("\n=== GNSS (nav) ===")
print(f"fix={n[44]} sats={n[45]} hdop={struct.unpack_from('<f',n,48)[0]:.2f} update_cnt={u32(n,68)}")
print(f"rmc={u32(n,80)} gga={u32(n,84)} zda={u32(n,88)} csum_err={u32(n,92)} field_err={u32(n,96)}")
