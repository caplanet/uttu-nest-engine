#!/usr/bin/env python3
"""cpswcore 디컴파일-복원 단서 감사기 (C-1..C-8).

빌드 후 바이너리를 스캔해 복원 단서가 남아 있으면 FAIL + exit 1 로 종료한다.
(빌드 게이트/CI 용) — 하드닝이 회귀하면 즉시 빨간불.

std 에 "내장(vendored)"되어 우리 RUSTFLAGS 로는 제거 불가한 크레이트 경로
(rustc-demangle, hashbrown 등)는 불가피하므로 허용목록으로 제외한다. 따라서
정상 하드닝 빌드는 PASS 하고, 프로젝트 의존성(clarabel, jagua-rs, i_overlay ...)
경로가 다시 새거나 다른 벡터가 회귀하면 FAIL 한다.

사용:
    python audit_leaks.py path\\to\\cpswcore.dll
"""
import io
import re
import sys

# cp949 콘솔에서도 한글/기호 출력이 깨지지 않도록 stdout 을 UTF-8 로 강제.
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

# std 에 vendored 되어 precompiled libstd 에서 나오는 크레이트들(우리 플래그로 제거 불가).
# -Z build-std 로 std 를 재빌드하지 않는 한 남는다 → 게이트에서 허용.
STD_VENDORED = {
    "rustc-demangle", "hashbrown", "adler", "adler2", "miniz_oxide", "gimli",
    "object", "addr2line", "memchr", "cfg-if", "libc", "unwinding", "std_detect",
    "compiler_builtins", "panic_unwind", "panic_abort", "allocator-api2",
    "rustc-std-workspace-core", "rustc-std-workspace-alloc", "rustc-std-workspace-std",
}

# 단순 정규식 검사: 하나라도 매치되면 FAIL.
REGEX_CHECKS = [
    ("C-1 로컬모듈경로", rb"swarm-core\\src\\|swarm-nfp\\src\\|third_party\\|\\grouped_engine\.rs|\\grouped_input\.rs|\\group_expansion\.rs",
     "로컬 크레이트 소스 경로/모듈 트리 노출 (location-detail 회귀?)"),
    ("C-2 cargo경로",  rb"\.cargo[\\/]registry",         ".cargo 레지스트리 절대경로 노출 (trim-paths 회귀?)"),
    ("C-2 rustup경로", rb"\.rustup[\\/]toolchains",      ".rustup 툴체인 절대경로 노출 (trim-paths 회귀?)"),
    ("C-3 PDB",        rb"[A-Za-z0-9_.-]+\.pdb",         "PDB 경로/이름 노출 (strip/DEBUG:NONE 회귀?)"),
    ("C-4 모듈경로",   rb"swarm_core::|swarm_nfp::",     "module_path!() 모듈 구조 노출 (로그 OFF 회귀?)"),
    ("C-7 알고리즘문자열", rb"\[STRUCT|\[NFP-|\[EXPL|\[DSRP|\[GROUPED|\[INIT\]|\[ESCAPE\]|\[ORIGINAL\]",
     "서술형 알고리즘 문자열 노출 (로그/패닉 메시지 회귀?)"),
    ("C-8 사용자명",   rb"mwlee",                        "빌드 PC 사용자명/호스트 지문 노출 (trim-paths 회귀?)"),
]

CRATEVER = re.compile(rb"([A-Za-z0-9_-]+)-\d+\.\d+\.\d+[\\/]src")

def check_crate_versions(data):
    """C-2: 크레이트-버전 경로. std-vendored 는 허용, 그 외(프로젝트 의존성)는 FAIL."""
    offenders = {}
    for m in CRATEVER.finditer(data):
        name = m.group(1).decode("latin1")
        if name in STD_VENDORED:
            continue
        offenders.setdefault(name, m.group(0).decode("latin1", "replace"))
    allowed = sorted({m.group(1).decode("latin1") for m in CRATEVER.finditer(data)} & STD_VENDORED)
    return offenders, allowed

def main(path):
    data = open(path, "rb").read()
    failed = False
    print(f"[audit] {path} ({len(data):,} bytes)\n")

    for label, pat, desc in REGEX_CHECKS:
        hits = re.findall(pat, data)
        if hits:
            failed = True
            uniq = sorted(set(hits))[:5]
            print(f"  FAIL  {label:14s} {desc}")
            for h in uniq:
                print(f"            e.g. {h.decode('latin1', 'replace')[:90]}")
            extra = len(set(hits)) - len(uniq)
            if extra > 0:
                print(f"            ... (+{extra} more unique)")
        else:
            print(f"  PASS  {label:14s} {desc}")

    offenders, allowed = check_crate_versions(data)
    if offenders:
        failed = True
        print(f"  FAIL  {'C-2 크레이트버전':14s} 프로젝트 의존 크레이트 이름+버전 노출 (location-detail 회귀?)")
        for name, sample in sorted(offenders.items())[:10]:
            print(f"            e.g. {sample[:90]}")
    else:
        note = f" (std-vendored 허용: {', '.join(allowed)})" if allowed else ""
        print(f"  PASS  {'C-2 크레이트버전':14s} 프로젝트 의존 크레이트 경로 없음{note}")

    print()
    if failed:
        print("RESULT: FAIL - 복원 단서가 남아 있습니다. Sources/Swarm/HARDENING.md 참고.")
        return 1
    print("RESULT: PASS - 프로젝트 복원 단서가 제거되었습니다 (std-vendored 잔여만 허용).")
    return 0

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
