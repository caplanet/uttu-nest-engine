# -*- coding: utf-8 -*-
"""G2 마커(.plt) -> Swarm 인스턴스 JSON 변환 드라이버.

argv 로 한글 경로를 넘기면 Windows 콘솔 코드페이지(cp949) 에서 깨지므로
경로를 소스에 유니코드로 박아 plt_to_swarm.main() 을 직접 호출한다.

변환 규약은 참조 파일 '여성WSjk_비교용-G_Swarm_PLT변환.json' 과 완전히 동일하다
(orientation=plt, hints 포함, 도메인 메타데이터는 CAD 유래 신규2 에서 이전).
같은 인자로 G.plt 를 돌리면 그 참조 파일이 바이트 단위로 재현된다 -- 검증 완료.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plt_to_swarm as P

PLT_G1 = r"C:\Caplanet\여성WSjk_비교용 여성WSjk_비교용 S(x1)-M(x1)-L(x1)-XL(x1) G.plt"
PLT_G2 = r"C:\Caplanet\여성WSjk_비교용 여성WSjk_비교용 S(x1)-M(x1)-L(x1)-XL(x1) G2.plt"
# 도메인 메타데이터(piece_name/size/grain/...) 공급원. PLT 에는 LB(텍스트) 명령이
# 0개라 라벨을 파싱할 수 없고, 이 CAD 유래 인스턴스에서 도형 매칭으로 이전한다.
REF_CAD = r"C:\Caplanet\AiLabs\Test Data Files\여성WSjk_비교용-G_New_Swarm_신규2.json"

OUT_DIR = r"C:\Caplanet\AiWorks\InputData"
OUT_G2 = os.path.join(OUT_DIR, "여성WSjk_비교용-G2_Swarm_PLT변환.json")
REPORT_G2 = os.path.join(OUT_DIR, "여성WSjk_비교용-G2_Swarm_PLT변환_매칭리포트.json")


def main():
    sys.argv = ["plt_to_swarm.py", PLT_G2, OUT_G2,
                "--reference", REF_CAD,
                "--report", REPORT_G2]
    rc = P.main()
    print("plt_to_swarm rc = %d" % rc)
    return rc


if __name__ == "__main__":
    sys.exit(main())
