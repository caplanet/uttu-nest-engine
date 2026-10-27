# 고정밀 입력의 Draft Init 지연 및 중지 후 표시 수정

## 재현

사용 입력: `E:/CaPlanit/여성WSjk_비교용-G_New_Swarm_신규2.json`.
156개 조각, 원본 33,477개 정점, mm 단순화 0.5mm, 겹침 허용 0.95mm, 힌트 제거.
오프셋 적용 후 정점은 49,962개이며 단순화 후 5,336개다.

수정 전 공개 Init API의 전체 시간은 86.986초, Draft 생성은 7.368초였다.
사용 길이 7560.217mm, 이용률 약 67.29%, 23개 열, 완료 배치 폴백 없이 156개 전부를 배치했다.
사용자가 제공한 첫 ID 순서, NFP 후보 실패 수 18개 및 예시와 동일한 결과를 재현했다.
Draft 로그의 시간에는 import/단순화와 이후 Init 검증이 포함되지 않아 실제 대기 시간이 숨겨져 있었다.

## 수정

- mm 단순화: 모든 후보의 전체 윤곽 복사를 지역 변경 정보로 대체하고, 원본 간선 이력의 역색인으로 해당 간선만 조회한다. 순위가 정해진 후보 중 실제 검사 대상만 윤곽을 구성한다. 누적 거리·보수적 방향·자기 교차 검사는 유지한다.
- WPF 로그: Init 중에도 네이티브 로그를 화면에 전달하고 경과 시간을 표시한다. 기초 기하 준비, mm 단순화/보조 도형 생성, 전체 Init, 초기 결과 표시의 시간을 구분한다.
- WPF 중지: Init 성공 후 초기 배치를 먼저 표시한 뒤 중지 여부에 따라 다음 phase를 생략한다. 기존에는 중지 확인에서 조기 반환해 완성된 초기 배치가 화면에 나타나지 않았다.
- 종료/새 실행: 오래된 실행 결과는 표시하지 않으며 네이티브 Init이 끝나기 전에 optimizer를 해제하지 않는다. Init API는 반환 전 핸들이 없어 이 구간의 중지는 완료 후 다음 phase를 막는 방식이다.

`[STRUCT:VALIDATE] ... rejected by depth-zero exact validation`은 quadtree의 충돌 보고를 정밀 재검증에서 기각했다는 의미다. NFP 위상 복원 실패 예시는 실패 후보를 제외했다는 진단이다. 이번 수정은 이 검증이나 NFP 실패를 우회하지 않는다.

## 수정 후 실측

동일 입력·설정에서 공개 Init 및 조회가 **86.986초 → 16.062초**로 감소했다(약 5.4배).

| 구간 | 수정 후 시간 |
|---|---:|
| 기초 기하 import | 2.472초 |
| mm 단순화 및 충돌 보조 도형 재생성 | 5.850초 |
| import 합계 | 8.322초 |
| Draft 생성 | 7.662초 |
| import·배치·검증을 포함한 네이티브 Init | 16.017초 |
| 공개 Init 및 결과 조회 | 16.062초 |

수정 전후 export된 전체 layout JSON이 같으며 모든 조각의 좌표·회전과 사용 길이 7560.217mm를 유지했다. 이 시간은 UI 렌더 시간을 포함하지 않는다. 실제 화면 표시 시간은 WPF의 `[Init] 초기 배치 표시 완료` 로그로 별도 확인한다.

공개 API 테스트 3개(합성 2개와 실제 입력), mm 기하 단위 테스트 3개, WPF 초기 표시 흐름 회귀 테스트 5개와 기존 ABI/설정 테스트 9개를 통과했다. WPF 흐름 테스트는 Init 중 중지 시 미리보기를 기다린 뒤 종료하는지, 폐기된 실행이 Init 완료까지 기다리되 표시하지 않는지, 실패/정상 결과를 구분하는지 확인한다.

기존 기하 기록 1248개(Inflate/Deflate, 여러 mm 한도와 겹침 설정)에 대해 최적화 전후 정점 좌표와 순서가 완전히 일치했다. 누적 홈 보존 및 import 옵션 회귀도 통과했다. WPF Release·Debug 빌드를 완료했고 두 출력 폴더에 갱신한 Release 네이티브 DLL을 배포했다. 기존 빌드 경고 14개는 남아 있다. 실제 WPF 창을 조작한 검증은 수행하지 않았으며 화면 표시·중지 순서는 분리된 production 흐름의 회귀 테스트로 확인했다.

## 재현 명령

```powershell
$env:DRAFT_INPUT='E:/CaPlanit/여성WSjk_비교용-G_New_Swarm_신규2.json'
$env:DRAFT_MM='0.5'
$env:DRAFT_OVERLAP_MM='0.95'
$env:DRAFT_SOLUTION="$PWD/output/draft-column/new-mm05-overlap095-after.json"
cargo test --release --offline --features 'simd,capmods,adaptive,structure-first' --test column_draft_ffi -- --include-ignored --nocapture --test-threads=1

$env:MM_EXPECTED_GEOMETRY="$PWD/output/draft-column/mm-geometry.json"
cargo test -p swarm-core --release --offline --features 'simd,capmods,adaptive,structure-first' --test mm_simplification -- --include-ignored --skip marker_mm_simplification_geometry --nocapture
```

시간은 빌드를 제외한 장비별 실측이며 10초 상한을 보장하지 않는다. 기존 비교용 PLT 변환 JSON의 측정과 이 고정밀 파일의 측정을 구분한다.
