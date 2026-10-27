#!/usr/bin/env python3
"""AU 실측 보고 — 밀도 % / 폭 mm / 시간 초 / 효율 %/s 를 한 표에 낸다.

엔진이 결과 JSON(`wsjk156.init.json`)에 `density`(0~1)와 `strip_width`(0.01mm)를
직접 써 준다. 그것을 읽어 «밀도·폭·시간·효율» 네 값을 한 줄로 보고한다.
"""
import json
import os
import re
import subprocess
import time

EXE = "./target/release/nesting-engine.exe"
IN = "./data/input/wsjk156.json"
OUT = "./data/input/wsjk156.init.json"


def run(name, args, timeout=280):
    if os.path.exists(OUT):
        os.remove(OUT)
    t0 = time.time()
    try:
        p = subprocess.run([EXE, IN] + args, capture_output=True, text=True,
                           timeout=timeout, encoding="utf-8", errors="replace")
        out = p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        print(f"{name:<38s}  TIMEOUT >{timeout}s")
        return None
    dt = time.time() - t0
    dens = float("nan")
    w = float("nan")
    if os.path.exists(OUT):
        try:
            with open(OUT, "r", encoding="utf-8") as f:
                res = json.load(f)
            dens = float(res.get("density", float("nan"))) * 100.0
            w = float(res.get("strip_width", float("nan")))
        except Exception:
            pass
    if dens != dens:
        m = re.search(r"최고 폭 Some\(([0-9.]+)\)", out)
        w = float(m.group(1)) if m else float("nan")
    eff = (dens / dt) if dens == dens else float("nan")
    ok = "OK" if dens == dens else "실패"
    print(f"{name:<38s} {w/100:>9.3f}mm  {dens:>7.3f}%  {dt:>6.2f}s  "
          f"{eff:>7.3f}%/s  {ok}")
    return w, dens, dt


if __name__ == "__main__":
    print(f"{'경로':<38s} {'폭':>11s}  {'밀도':>8s}  {'시간':>7s}  "
          f"{'효율':>9s}")
    print("-" * 86)
    run("A) 기본", [])
    run("B) AU160최적 slack+scan-score r6",
        ["--canonical-slack", "--canonical-scan-score", "--canonical-rounds", "6"])
