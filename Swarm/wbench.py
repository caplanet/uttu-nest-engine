"""AU199 — canonical_window 크기별 실측. 창 제거가 원천과 일치하는지 확인."""
import json, os, subprocess, time

EXE = "./target/release/nesting-engine.exe"
IN = "./data/input/wsjk156.json"
OUT = "./data/input/wsjk156.init.json"


def run(name, args, t=200):
    if os.path.exists(OUT):
        os.remove(OUT)
    t0 = time.time()
    try:
        p = subprocess.run([EXE, IN] + args, capture_output=True, text=True,
                           timeout=t, encoding="utf-8", errors="replace")
        out = p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        print(f"{name:<46s} TIMEOUT>{t}s")
        return None
    dt = time.time() - t0
    d = float("nan"); w = float("nan")
    if os.path.exists(OUT):
        try:
            r = json.load(open(OUT, encoding="utf-8"))
            d = float(r.get("density", float("nan"))) * 100.0
            w = float(r.get("strip_width", float("nan")))
        except Exception:
            pass
    eff = (d / dt) if d == d else float("nan")
    print(f"{name:<46s} {w/100:>9.3f}mm {d:>7.3f}% {dt:>6.2f}s {eff:>7.3f}%/s", flush=True)
    return d, dt


print(f"{'경로':<46s} {'폭':>11s} {'밀도':>8s} {'시간':>7s} {'효율':>9s}")
print("-" * 94)
BASE = ["--autonest-native", "--time", "150"]
run("기준 window=8 (AU190 76.361%)", BASE)
for w in (24, 64, 9999):
    run(f"window={w}", BASE + ["--canonical-window", str(w)])
