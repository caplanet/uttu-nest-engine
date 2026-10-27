# R&R 선택 통계와 빈 Polish 작업 제거

2026-09-09 · `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`

기존 설정값과 알고리즘 사용 의미를 유지하면서, 실행하지 않을 작업에 대한 계산과 준비를 줄였다. 신규 Config 옵션이나 컴파일 기능 변경은 없다. 앞서 거부한 원본 겹침 감소 실험은 연결하지 않았다.

## 적용 내용

### R&R 선택 통계

`optimizer/explore_mods.rs`의 연산자 선택 전에 R&R 활성·적용 가능 여부와 실행 주기를 확인한다. R&R을 선택할 수 없는 회차에는 이동 조각/충돌 비율, 충돌 중심, 실패 해 풀 분포, 밀도 등 상세 선택 통계를 만들지 않고 기존 Swap 교란을 선택한다.

R&R 회차에서는 기존 선택기를 그대로 사용한다. 각 연산자의 점수, 주기, Swap fallback, no-op 검사, 후속 separation, 정밀 인증은 유지한다. 폭 복구 등 다른 알고리즘이 사용하는 충돌 중심 통계는 제거하지 않았다. 연산자 적용 이후의 공통 성과 계측도 유지한다.

비활성·비주기·활성 주기 조합에서 상세 context 생성 호출 횟수와 기존 선택 결과/점수가 일치하는지 테스트했다. R&R을 완전히 끄면 선택용 context 호출은 0회다. 주기 3이면 선택 가능한 12회 교란 중 4회만 context를 만든다. 이는 호출 수 검증이며 전체 Explore 실행 시간의 개선율 측정은 아니다.

### 빈 Polish

`has_active_passes`가 기존 Polish/LP/Slide/Hole Fill 설정과 PlaceGroup 제약을 함께 판단한다.

- **Legacy 단계 API:** 입력을 인증한 뒤, 실행할 패스가 없고 Reclaim도 꺼져 있으면 Separator·워커·pool을 만들지 않는다. 기존 problem을 인증된 solution으로 맞추고 Final callback·GetSolution·Polish 단계 전이는 유지한다. phase seed 소비 경계도 보존한다.
- **Grouped:** 모든 lane에서 패스를 실행할 수 없으면 공개 결과를 인증하고 단계만 진행한다. 이 경우 lane transaction 복제, 스케줄러 pool, 반복 slice 및 Separator 생성을 생략한다. 활성/비활성 lane이 섞인 경우에는 인증된 비활성 lane이 자체 Separator를 만들지 않는다.
- **Direct core optimizer:** Compress에서 이미 인증한 입력에 적용할 패스가 없으면 로컬 Polish 호출을 생략한다. 독립적인 Reclaim 분기와 예산은 유지한다.
- **Core Polish 함수:** 직접 호출자가 넘긴 입력의 기존 복원·원본 보정·인증 절차를 거친 뒤, 적용할 패스가 없으면 라운드 0회로 반환한다. 미인증 입력까지 무조건 정상 결과로 취급하지 않는다.
- NFP Hole Fill의 빈 cache 객체는 실제 Hole Fill 패스를 사용할 때만 만든다.

**Polish OFF가 Reclaim OFF를 의미하도록 바꾸지 않았다.** Legacy의 Reclaim ON 경로는 유지하고, 테스트에서 실제 사용 길이 감소를 확인했다. Grouped 엔진은 원래 별도 Reclaim 호출이 없는 경로이며 이번 작업에서 실행 기능을 새로 추가하거나 제거하지 않았다. Structure P5의 rotation snap/분리 경로도 유지한다.

일반적인 worker/pool 지연 생성, Conflict Burst의 NFP 준비, Raster OFF 최소 치수 순회 제거는 이번 변경에 포함하지 않았다.

## 검증

- Core 단위 테스트: **732 통과**, 기존 ignored 13개는 실행하지 않음.
- Core 후처리 통합 테스트: **10 통과**.
- Native 라이브러리 단위 테스트: **101 통과**.
- 추가 FFI 통합 테스트: **2 통과**. 내부 조합은 Legacy/단일 그룹/여러 구간 × Polish OFF/패스 없음 × Final callback에서 중지 요청 유무를 포함한다.
- 빈 Polish의 입력/최종 공개 배치 일치, 잠금·그룹 좌표 유지, mandatory Final JSON, 단계 호출 순서 및 재호출 거부 확인.
- 그룹 때문에 LP가 실행 불가인 경우도 빈 패스로 처리하고, 정상 배치를 보존함.
- 로컬 Polish OFF + Reclaim ON에서 원단 길이가 실제 감소함.
- 기존 활성 후처리 및 callback 직후 중단 시 마지막 인증 결과 보존 테스트도 통과.
- `git diff --check` 통과.

명령:

```powershell
cargo test -p swarm-core --lib --test postprocess_phases --features "simd capmods adaptive structure-first"
cargo test -p swarm --lib --test disabled_work --features "simd capmods adaptive structure-first"
cargo build --release --example disabled_polish_bench --features "simd capmods adaptive structure-first"
```

## 성능 비교 범위

`examples/disabled_polish_bench.rs`로 156개 사각형을 초기화한 뒤, 압축 완료 상태에서 public `swarm_refine_phase` 호출 시간만 비교한다. 초기화·Compress·호출 전후 GetSolution JSON 읽기는 시간 측정에서 제외한다. 각 설정 12회, Final callback 미등록, 기존 Release/fat LTO 프로필을 사용한다. 전후 공개 배치와 원단 길이의 동일성을 함께 검사한다.

전체 네스팅 시간이나 수율 개선을 측정한 벤치마크가 아니다. 형상이 복잡하거나 Grouped 경로이면 절감량이 달라질 수 있다. 기존 인증 cache와 구조체 공유 최적화가 있어, 소스상 작업 제거 비율과 시간 감소율이 같지는 않다.

| 조건 | 변경 전 중앙값 | 변경 후 중앙값 | 호출당 절감 |
|---|---:|---:|---:|
| Polish OFF, Reclaim OFF | 2.1915ms | 0.03885ms | 2.15265ms (약 98.2%) |
| Polish ON, LP/Slide/Hole Fill OFF, Reclaim OFF | 2.3477ms | 0.02890ms | 2.31880ms (약 98.8%) |

Release 벤치 빌드와 두 측정 실행 모두 성공했다. 원본 JSON 및 각 12회 집계는 `target/disabled-work-before.jsonl`, `target/disabled-work-after.jsonl`에 있다. 이 자료에서는 절대 절감량이 빈 호출 1회당 약 2ms다. 전체 네스팅을 98% 빠르게 만들었다는 의미가 아니며, 반복 호출 수와 R&R 통계 비용에 따라 실제 전체 효과를 별도로 측정해야 한다.

## 변경 파일 및 보관

- core: `optimizer/adaptive_selector.rs`, `optimizer/explore_mods.rs`, `optimizer/polish/mod.rs`, `optimizer/mod.rs`.
- native: `src/ffi_step.rs`, `src/grouped_engine.rs`.
- 검증: `swarm-core/tests/postprocess_phases.rs`, `tests/disabled_work.rs`, `examples/disabled_polish_bench.rs`.
- 변경 전 주요 소스 6개 보관: `target/disabled-work-before-20260909-122410`. 기존 다른 작업의 변경을 포함한 수정 직전 사본이며 저장소 전체 백업은 아니다.

이번 작업에서 WPF 코드는 수정하지 않았고 WPF 빌드·실행 또는 배포 DLL 교체는 하지 않았다. Rust 빌드 산출물과 검증 로그는 이 저장소의 `target`에 있다.
