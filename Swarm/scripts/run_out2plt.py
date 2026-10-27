# -*- coding: utf-8 -*-
"""드라이버: 한글 경로를 argv 로 넘기면 Windows 콘솔 코드페이지에서 깨지므로
경로를 유니코드 리터럴로 박아 swarm_output_to_plt.main() 을 직접 호출한다.

두 벌을 만든다.
  1) 조각 윤곽 + 테두리만  -- 배치 결과 JSON 에 대한 순수 변환(기본 산출물)
  2) 1) + 원본 마커의 조각별 주석(라벨/결방향/너치) -- 눈으로 조각을 식별하기 위한 보조본.
     원본에서 라벨이 조각 밖까지 뻗어 있어(최대 85mm) 새 배치에서는 옆 조각 위로
     겹쳐 그려진다. 재단용이 아니라 확인용이다.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import swarm_output_to_plt as S

RESULT = r"C:\Caplanet\AiWorks\InputData\output\여성WSjk_비교용-G_New_SwarmV1_output_20260908_130417.json"
INST = r"C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json"
SRC_PLT = r"C:\Caplanet\여성WSjk_비교용 여성WSjk_비교용 S(x1)-M(x1)-L(x1)-XL(x1) G.plt"
OUT_DIR = r"C:\Caplanet\AiWorks\InputData\output"
BASE = "여성WSjk_비교용-G_New_SwarmV1_output_20260908_130417"

rc = 0
for suffix, src in ((".plt", None), ("_라벨포함.plt", SRC_PLT)):
    argv = ["swarm_output_to_plt.py",
            "--result", RESULT,
            "--instance", INST,
            "--output", os.path.join(OUT_DIR, BASE + suffix)]
    if src:
        argv += ["--source-plt", src]
    sys.argv = argv
    rc |= S.main()
sys.exit(rc)
