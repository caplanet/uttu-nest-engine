# Swarm 작업 내역

- 원본: 기존 저장소 (정리한 커밋 63개, 2026-09-30 ~ 2026-10-23)
- 이관: 2026-10-27 `uttu-nest-engine/Swarm` (커밋 `b118f18`, Swarm v1.0.3) = 원본 최종 커밋 `385fa8c`

## 버전 요약

| 버전 | 날짜 | C ABI | 핵심 |
|---|---|---|---|
| 1.0.0 | 09-30 | v18 | 기준선: 탐색 정체 실측, Stop 응답 개선, DLL 이름 `cpswcore` |
| 1.0.1 | 10-06 | v18 | 겹침 프록시 감쇠 복원(+1.44%p), Polish LP 개선, 단계 자유 순서, 진단 로그 제거 |
| 1.0.2 | 10-15 | v19 | 10-13 전면 코드 리뷰, 원본 겹침 허용 옵션, `build.bat` |
| 1.0.3 | 10-23 | v20 (+ 입력 ABI v1) | jagua-rs 1.1.0 포크·멀티파트 아이템, 릴리스 보안 강화(cpguard), 구조체 입력, Init 최적화 |

## 1.0.0 기준선 (2026-09-30)

- 저장소 생성(`735e36e`), 기존 Swarm 엔진 반입(`497b556`)
- 탐색 정체 실측(156조각 마커 240초): 분리 재출발 창·분리 시간 상한·교란 포트폴리오를 구현해 3시드 측정 → 이득 없어 기본 끔
- Stop 응답
  - Init: 핸들 생성 전 전역 취소 `swarm_cancel_init()` (오류 2002), WPF StopAsync 연결
  - Polish: microlp 데드라인 검사 1000 → 32 pivot, 정지 3.4초 → 0.03~0.1초
- ABI v18 (설정 구조체 952바이트), DLL 이름 `capswarmcore_100` → `cpswcore`
- 문서: `docs/EXPLORE_STALL_AND_STOP_2026-09-30.md`

## 1.0.1 (2026-10-01 ~ 10-06)

### 품질
- 겹침 프록시 `pd <= 0` 클램프 제거(`97ae9ab`): 얕은 충돌의 기울기를 복원해 상류 구현·논문과 같은 정의로 되돌림
  - wsjk156 (겹침 0.4mm, 480/60/60초) 3시드 평균 79.065% → 80.503% (+1.438%p, 3/3 승)
  - 주의: 손실 스케일이 바뀌었다. 이전 문서의 loss 수치와 «기각» 판정은 재측정 전에는 근거로 쓰지 않는다
  - 문서: `docs/OVERLAP_PROXY_DECAY_2026-09-30.md`
- Polish LP(`e474863`): 접촉 행을 선분 기준으로, 폭 변수를 우측 끝 기준으로, 좁은 접촉의 마진 복원, 막힘 시 패스 종료
  - 같은 Compress 배치에서 Polish 이득 +0.04~0.07%p → +0.26~0.46%p, 전체 실행 seed 1 최종 80.978%
- 탐색 정체·Reclaim·원단 경계 보정 수정, 시드 대결(짧은 탐색 뒤 폭으로 NFP-LBF 시드 채택)(`3985605`)
- 충돌 해소 단순화·저코어 오버헤드 감소, GLS 갱신 벡터화, Polish 가 시간 예산을 끝까지 사용(`4346371`, `1a9979c`)

### API·호스트
- 단계 API: Explore/Compress/Polish 를 임의 순서·반복으로 호출 (Structure 중에는 거부, 오류 3000)
- WPF: fsn UI 이식 (다중 시드 최대 6개, 설정 프리셋, 실행 순서 편집기, 리본 레이아웃)
- C 헤더 `swarm.h` 를 ABI v18 로 동기화
- 문서: `docs/HANDOFF_2026-10-01.md`, `docs/HANDOFF_2026-10-01_B.md`

### 정리
- CPU 만 쓰는 진단·로그 제거 (최적화 결정·RNG·결과 불변), 진행 콜백 기본 간격 2초(`4338d72`)
- 제품 버전 1.0.1

## 1.0.2 (2026-10-07 ~ 10-15)

### 기능
- Polish NFP 왼쪽 재배치 (기본 켬, 평균 길이 −0.64mm). 비스듬한 slide·원본 외곽 인지 slide 는 옵션(기본 끔)(`c727c87`)
- Structure Polish 진행 보존, Legacy 초기 배치의 원본 외곽 인증, Init 전 구간 취소·동시 시드 시작(`d1ee8dc`, `0c0e263`, `1e83298`)
- PlaceGroup 내부 겹침 보존, 취소 전파, 실패 시 마지막 인증 해 유지, 그룹 CPU 예산 준수(`8ce8874`, 이슈 #1~#11)
- 원본 외곽 초과 겹침 허용 옵션 `OriginalOverlapPolicy` (ABI v19, 구조체 크기 952바이트 유지)(`90c35d3`)
- `build.bat` 추가: release/debug, x64/arm64, rust/wpf, test 선택(`0a294c6`)

### 2026-10-13 전면 코드 리뷰 (커밋 약 30개)
- 경계 강화: C ABI·JNI·wasm 입력 검증, step FFI 의 잠금·힌트·수명 버그, 콜백 안 `swarm_free` 교착 → stop 요청 후 핸들 누수로 변경
- 버그 수정
  - Init: attach 무한 루프·아이템 누락·회전 손실, 참조 배치 앵커·충돌 검사, 비유한 가중치 거부
  - Structure: clamp 패닉, P0 예산, 고정 아이템 웜스타트
  - NFP-LBF: prefix 재사용 오류, 장애물 누락, IFP 캐시 무한 증가
  - 탈출 파이프라인이 실제로 교란하지 않던 것, raster 후보의 스트립 밖 돌출, Xpress 제어 ID 오류
  - nesting-engine: 원본 외곽 이중 회전, 읽히지 않던 웜스타트
- 웜스타트: 가장자리에 붙은 힌트 하나 때문에 156개 배치가 전부 버려지던 것 수정(2mm 이내면 안쪽으로 이동), 읽을 수 없는 해에 경고
- Polish 가 고정점을 증명하면 바로 끝내 Reclaim 이 남은 예산을 쓰도록 변경
- 성능: 분리기·압축·평가기 오버헤드 감소, SIMD 커널 디스패치, swarm-nfp 중복 기하 연산 제거
- 테스트: all features 1257 / default 1112 통과, 실패 0
- 문서: `docs/CODE_REVIEW_2026-10-13.md` (판단 기록과 되돌리는 방법 포함)

### 제품 버전 1.0.2 (`1cd35fc`)

## 1.0.3 (2026-10-15 ~ 10-23)

### PlaceGroup·멀티파트 아이템
- 그룹 멤버 목표를 초기 상대 자세에서 f64 로 계산 → f32 누적 드리프트로 인증이 실패하던 것 수정(`b6f7119`)
  - 문서: `docs/GROUP_WORKERS_912_2026-10-15.md` (#912 그룹 입력의 1 vs 8 워커 결과 차이 분석)
- jagua-rs 1.1.0 포크를 `third_party/jagua-rs` 에 벤더링, 변경점은 `CAPLANET_CHANGES.md`(`dc7b05f`)
- 멀티파트 아이템 지원 → 파트별 충돌 정량·인증 → V2 PlaceGroup 을 멀티파트 아이템으로 네스팅(`8ccc959` ~ `6c73497`)
  - `place_group_mode` MultiPolygon 기본(Rigid 는 비교용), ABI v20, 구조체 952 → 960바이트
  - #912 측정에서 모든 체크포인트가 Rigid 보다 좁아 기본값으로 유지
  - 콜백 충돌 상태·인스턴스 면적을 파트 기준으로 수정 (이슈 #20, #21)
  - 문서: `docs/MULTIPOLYGON_ITEMS_PLAN_2026-10-16.md`

### 보안 강화 (정적·동적 분석 대응)
- 릴리스 빌드 정보 유출 차단(`27c42d6`): trim-paths, 심볼 strip, PDB 미생성, 릴리스에서 로그 매크로 제거, panic 메시지 419곳 `dmsg!` 처리, `scripts/audit_leaks.py` 검사
- 버전 리소스에서 알고리즘·제품 설명 제거(`433aca3`)
- cpguard micro-VM 으로 시드 도메인 상수 은닉 → cpguard v0.2 로 갱신(`aafe2d7`, `902e1cd`)
- cpguard 런타임 안티탬퍼: 옵트인(기본 끔), 크래시 없이 콜백으로만 보고, 첫 스캔 약 15ms (PR #22)(`a1490c5`)
- Uttu Nest 연동 정리: cpguard 적용, 동적 분석 시 단계 명칭 변경 (#962)(`166e714`)

### 기타
- WPF 다중 시드에서 Grouped 엔진의 Explore 중간 해 표시(`25a8905`)

### Swarm 1.0.3 (`385fa8c`)
- 구조체 직접 입력 C ABI 추가 (`SWARM_INPUT_ABI_VERSION` 1), 기존 JSON 입력 유지, 공통 입력 검증
- Init 최적화: BVH 로 충돌 후보 축소, 점수 기반 조기 제외
- 핵심 전략·정책 함수 6곳에 cpguard 적용, cpguard 유무 빌드 분리(`build_cpguard.bat`, `build_no_cpguard.bat`, `docs/CPGUARD_BUILDS.md`)
- 제품·Cargo 버전 1.0.3, C 헤더 ABI v20 동기화
- 벤치마크 코드는 포함하고 측정 결과·보고서·로그는 Git 에서 제외
