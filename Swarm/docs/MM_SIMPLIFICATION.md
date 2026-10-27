# 최대 이탈(mm) 단순화

WPF 설정의 **최대 이탈(mm)** 체크박스를 켜고 양수를 입력한다. 입력값 기본은 0.5mm이며 체크박스 기본은 꺼짐이다. 현재 입력의 Draft 실측에서 0.5mm는 시간 초과 없이 완료했고, 0.3mm는 더 정밀하지만 탐색 예산을 소진했다. 켜면 `PolySimpl` 입력은 비활성화되고 면적 비율 단순화와 좁은 홈 닫기를 대체한다. 끄면 기존 설정을 그대로 사용한다.

`Draft Init`과 독립적인 전역 import 옵션이다. legacy 입력 및 grouped 입력의 전체/레인 import에 연결했다. Draft에서는 mm 모드가 만든 충돌 도형에 추가 면적 비율 단순화를 하지 않고 NFP를 준비한다. 새 배치를 비교할 때는 `Use Hints`를 끈다.

## 의미

- 한도는 **명시한 간격 또는 겹침 오프셋을 적용한 직후의 경계** 기준이다. 오프셋 자체의 변화량을 이 mm 값에 포함시키거나 상쇄하지 않는다.
- 조각은 바깥쪽으로만 단순화(`Inflate`), 일반 컨테이너용 함수는 안쪽으로만 단순화(`Deflate`)한다. 스트립 컨테이너는 직사각형이므로 추가 단순화가 필요 없으며 기존 경계/간격 처리를 유지한다.
- 면적 증가율이나 마지막 한 번의 수정량이 아니라 **처음 경계에 대한 누적 양방향 경계 거리**를 제한한다. 여러 번 병합해도 기준 경계를 새 결과로 교체하지 않는다.
- 원본 재단 윤곽과 export 좌표는 보존한다. 단순화 도형은 충돌 판정/탐색에만 사용한다.
- 단순화 한도를 실제 겹침 허용으로 해석하지 않는다. 겹침은 기존 `겹침 허용(mm)` 값으로 별도 지정한다.

## 구현

`swarm-core/src/util/simplify_mm.rs`는 Jagua와 같은 세 종류의 기하 연산을 사용한다: 오목 코너 삭제, 공선 코너 삭제, 연속한 볼록 코너 두 개를 연장선 교점으로 병합. 일반 Douglas–Peucker를 사용하지 않는다. Deflate는 순회 방향을 뒤집어 같은 연산의 보수적 방향을 반대로 적용한다.

각 간선은 흡수한 **원본 간선 번호**를 보관한다. 후보가 일부 간선을 대체할 때 이 이력을 합친다. 원본 정점만의 수직거리 검사는 선분 중간의 이탈과 볼록 코너의 긴 교점 돌출을 놓칠 수 있어 다음 두 검사를 추가했다.

1. 새 선분 전체가 관련 원본 선분들에서 최대 mm 이내에 있는가.
2. 영향을 받은 원본 선분 전체가 그 이력을 가진 현재 선분들에서 최대 mm 이내에 있는가.

선분의 허용 거리 영역은 양 끝 원과 가운데 직사각형을 합친 캡슐이다. 선분 매개변수의 허용 구간을 해석적으로 계산하고 구간 합집합이 `[0,1]`을 덮는지 검사한다. 정점 샘플링으로 수용 여부를 결정하지 않는다. 후보의 순서는 정점 거리에서 얻은 이탈 하한으로 비교하고, 수용은 위 연속 경계 인증으로 결정한다. 새 교점은 실제 저장할 f32 좌표로 바꾼 뒤 인증하며, 보수적 방향 및 자기 교차도 검사한다.

원본 간선 이력은 불변 공유 저장소를 사용해 후보마다 전체 이력 벡터를 복사하지 않는다. 기하 커널/Jagua의 설치본은 수정하지 않았다.

고정밀 입력에서 모든 후보마다 전체 윤곽을 복사하던 비용도 제거했다. 후보 순위는 지역 변경 정보와 원본 간선→현재 간선의 역색인으로 계산하고, 실제 연속 경계/자기 교차 검사에 들어갈 후보만 전체 윤곽으로 만든다. 후보 점수·선택 순서·누적 이탈 인증은 유지한다. import 로그는 기초 기하 준비와 mm 단순화/충돌 보조 도형 생성 시간을 구분한다.

## 설정 및 ABI

- Rust: `SwarmConfig.simplify_max_deviation_mm: Option<f32>` 및 `import_spp_instance_with_mm(...)`.
- C#: `SwarmConfig.SimplifyMaxDeviationMm: float?`.
- C ABI: `refplace.simplify_max_deviation_mm: float`는 기존 `CRefPlaceConfig`의 마지막 4바이트 예약 슬롯을 사용한다. `refplace.enabled`와 무관하게 적용한다. 0은 비활성, 유한한 양수는 활성이다. 전체 776바이트 크기와 기존 필드 오프셋을 유지했다.
- 기존 `poly_simpl_tolerance`의 단위와 의미는 바꾸지 않았다. mm 모드가 우선하며 별도 함수의 `Some(0)`은 단순화 없이 오프셋만 사용하는 검사/연동 용도다. WPF/C ABI에서 활성화할 값은 양수다.
- NaN/무한대/음수는 코어 함수에서 오류로 반환한다. 관리 설정에서 비양수 활성 값은 오류이며 네이티브 옵션 해석에서는 비활성으로 처리한다. 기본 설정 조회는 mm 슬롯도 명시적으로 초기화한다.

## 검증과 재현

연속 선분 중간의 거리, 스케일이 달라져도 고정 mm 한도를 유지하는 경우, 오목/볼록 병합, Deflate, 누적 오차, 면적 모드 대체, 오프셋 유지, ABI 크기 및 기본값 재설정을 검사한다.

```powershell
$env:DRAFT_INPUT='E:/CaPlanit/여성WSjk_비교용-G_Swarm_PLT변환.json'
$env:MM_GEOMETRY_REPORT="$PWD/output/draft-column/mm-geometry.json"
cargo test -p swarm-core --release --offline --features 'simd,capmods,adaptive,structure-first' --test mm_simplification -- --include-ignored --nocapture --test-threads=1
python output/draft-column/audit_mm_geometry.py

$env:DRAFT_MM='0.3'
$env:DRAFT_OVERLAP_MM='0.1'
$env:DRAFT_SOLUTION="$PWD/output/draft-column/mm03-overlap01-solution.json"
cargo test --release --offline --features 'simd,capmods,adaptive,structure-first' --test column_draft_ffi real_marker -- --ignored --nocapture --test-threads=1
```

별도 GEOS/Shapely 검사에서 source/result 포함 관계와 세밀하게 보간한 양방향 경계 거리를 확인한다. 실제 수용 판정은 Rust의 연속 경계 인증이다. 비교용 JSON의 f32→십진수 표현 오차는 별도로 기록한다. 현재 파일은 이미 9,194개 정점으로 단순화된 입력이므로, 참고 대화의 33,477개 정점 입력에서 나온 감소율/예상 시간을 그대로 적용하지 않는다.

단순화 단계가 빨라지거나 정점 수가 감소해도 최종 Init 밀도/시간의 단조 개선은 보장하지 않는다. 특히 더 충실한 NFP가 기존의 강한 면적 단순화 제안보다 비쌀 수 있다. 추적되는 실측은 [측정 기록](MM_MEASUREMENTS_2026-09-07.md)에 보존한다. 원시 검증 결과는 로컬 `output/draft-column/mm-geometry-audit.json`에 있다. LP 압축 후속안과 현재 한계는 [진행 기록](DRAFT_PROGRESS_2026-09-07.md)을 참조한다.
