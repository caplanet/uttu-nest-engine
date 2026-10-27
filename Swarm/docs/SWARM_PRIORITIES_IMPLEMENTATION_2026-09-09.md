# Swarm 우선순위 1·2·3·4·5·9·10 구현 기록

작업 경로는 `C:/Caplanet/AiWorks/AdaptiveAlgorithm2`다. Uttu.Nest와 fable 소스는 읽거나 수정하지 않았다. 설정값이나 컴파일 기능을 추가해 대체하지 않고 실제 계산·실행 경로를 변경했다.

**후속 요청 반영:** 노력·남은 시간 기반 shrinkStep과 실패 직후 쉬운 목표 길이 재시도는 롤백했다. 기존 밀도/Feedback 계산은 복원했고, 조기 인증 콜백과 중지 시 인증 결과 보존은 유지했다.

선택한 일곱 항목과 shrink 롤백의 구현·검증·저장소 WPF 빌드를 완료했다. 실제 마커 전체 실행의 성능 일반화 범위는 아래 측정 절에서 구분한다.

## 적용 내용

| 순서 | 변경 | 동작 및 보호 조건 |
|---:|---|---|
| 1 | 실제 NFP 전략에 맞는 비용 추정 | Accretion → Best-fit → Beam → Greedy 실행 우선순위와 같은 선택 함수를 사용. 배타적인 Best-fit·Beam 비용을 함께 곱하지 않는다. 실제 후처리 비용은 유지. |
| 2 | 회전 bbox와 형상 계수 재사용 | LBFBuilder/Separator 생성당 한 번 준비하고 워커가 공유. 연속 회전 16개, 이산 회전 순서·중복·부호 있는 0, 난수 소비, f32 연산 순서 유지. 형상·회전 변경은 새 준비로 대체. |
| 3 | 원본 검사 첫 위반 반환 | 인증은 첫 위반에서 중단. 보정·진단은 전체 쌍 유지. SIMD bbox 선별과 유효 후보의 제한된 병렬 계산 유지. 무한대 좌표가 행렬 계산 패닉을 일으키기 전에 오류로 반환. |
| 4 | full/lane 준비 형상 공유 | lane의 local ID와 수량만 재구성하고 원본/CD 형상 Arc 공유. lane별 원단 길이 및 음수 간격의 원단 축소 상한은 다시 계산. |
| 5 | 인증해 기반 Init 경쟁·부분 개선 | 원본 보정·인증 뒤 생성기 후보를 비교. 부분·충돌·길이 악화 후보 기각. 별도 Initial은 기존 인증 배치에서 시작하며 잠금/그룹/연속 회전은 이동 가능한 단위만 개선. |
| 9 | 강체 그룹 LP | 그룹당 공통 dx/dy, 멤버별 경계와 외부 접촉. 내부 접촉은 제약 수 제한 전에 제외. 잠금 멤버가 있으면 그룹 전체 고정. 적용 후 원본 인증과 실패 복원. |
| 10 | 소수 조각 공동 pocket 배정 | 우측 강체 단위 2~4개, 단위별 최대 7후보(현재 위치 포함), 최대 4포켓·4,096개 탐색 노드. 후보 간 정밀 충돌을 검사한 뒤 하나의 이동으로 적용·원본 인증. 기존 개별 raster/NFP 작업을 이어간다. |

명시적으로 선택한 Column Draft의 새 배치 생성 경로는 유지했다. 원본 겹침 허용은 기존 `piece_overlap_mm` 한도를 그대로 사용한다. 잠금/잠금 겹침과 잠금/원단 이탈 예외, 원본 인증, OrgID와 좌표 변환 계약도 유지한다.

## 코드 리뷰에서 함께 수정한 문제

- LP 내부에서 인증한 개선이 후속 반복 중 Stop으로 끝나면 Polish가 더 오래된 결과로 복원하던 경로를 수정했다. 재검사 없이 이미 인증한 LP 결과를 보관한다. 재현 테스트에서 이전 코드는 22.800228mm 인증 결과 대신 32mm를 반환했다.
- Initial의 GLS 축소를 인증하고도 선택적 재압축이 끝날 때까지 보관하지 않던 경로를 수정했다. 축소 성공을 먼저 보관·보고하고 재압축 실패/중지에는 그 결과를 유지한다.
- 원본 보정 전에 LBF 길이를 다른 생성기와 비교하던 순서를 변경했다. 보정으로 길이가 늘 수 있으므로 인증 후 길이를 비교하며, 인증해가 없으면 유효한 완전 후보를 우선 확보한다.
- hard escape 뒤 tracker 복구 경로도 준비된 형상 계수를 사용하도록 연결했다.
- 공동 Hole Fill의 개선을 Polish가 다른 동점 기준으로 버리던 연결을 수정했다. 공동 배정이 가능한 설정에서는 LP 중간 보관·공동 checkpoint·라운드 최종 채택에 같은 강체 단위별 우측 끝 순위를 사용한다. 원단 길이가 최우선이며, 같으면 큰 우측 끝부터 순서대로 비교한다.
- 원단 경계에 정확히 닿은 유효 후보는 기존 mm 경계 검사와 같은 기준으로 처리한다. 별도 그룹의 공개 ID가 중복돼도 대표 조각 기준으로 구분한다. 그룹별 우측 끝은 snapshot 한 번 순회에서 집계한다.

## 검증 기록

중복되는 실행이 있으므로 다음 행의 통과 개수를 합산하지 않는다.

| 범위 | 확인된 결과 |
|---|---|
| NFP 전략 선택/비용 | 5개 통과, 주요 전략 플래그 8조합 포함 |
| 첫 원본 위반 반환 | 3개 통과: 첫 쌍·전체 순서 동일, 잠금 예외, 취소/비정상 좌표, 반복 캐시 |
| 준비 lane | 3개 통과: 면적/mm 단순화, 재import와 형상·pose·strip 동일, Arc 공유, local ID/수량, 작은 원단 상한 |
| 형상 준비 생산 기능 조합 | 27개 회귀 테스트, SIMD 경계 테스트 6개 통과 |
| 그룹 LP | 7개 통과: 유효 압축, 그룹/잠금/회전 보존, 충돌 거부·backtracking, 중간 반복 Stop과 콜백 결과 보존 |
| Rust FFI 및 V2 lane | 최종 native lib 104개 + 공개 API 통합 21개 통과. 그룹 15개·잠금 4개·OrgID 2개 포함. 실패 0. |
| 전체 core 및 관련 통합 | 최종 생산 기능: core lib 753개 + 관련 통합 63개 통과, 실패 0. 명시적 제외 15개(기존 실험·벤치 포함). shrink/폭 복구/조기 보고, 원본·겹침·잠금·인증 캐시 회귀 포함. |
| SIMD 없는 조합 | 준비/Init/LP/원본/lane 24개, joint·postprocess 17개, holes lib 11개 통과. 행 사이 일부 범위 중복. |
| Rust Release / WPF x64 Release | 최종 DLL은 cdylib 단독 fat LTO 빌드 성공. WPF 경고 0·오류 0. 복사된 DLL의 SHA256 동일, 실제 로딩 후 ABI·33개 필수 export 확인. |

최종 core 로그: `target/all-phase-priorities-validation-20260909/core-production.log`. 생산 core 기능은 `simd capmods adaptive structure-first`이며, 기본 조합은 `--no-default-features`로 확인했다. 새로운 기능 시험뿐 아니라 기존의 잠금 예외·2mm 겹침·Compress/Polish 인증 계약도 함께 실행했다.

## 측정의 범위

- 동일한 연속 회전 16개로 sampler를 20,000회 생성·샘플링한 한 번의 분리 측정: **21.913ms → 3.687ms**, checksum 동일. 반복 준비 구간 약 5.94배이며 전체 Explore 속도 향상률이 아니다. 재사용 경로의 불변 준비는 반복 측정 전에 한 번 수행했다.
- 공동 Hole Fill의 작은 함정 형상: 초기 32mm → 순차 Hole Fill 19.004221mm → 공동 배정 11.010111mm. raster/NFP 두 경로에서 두 단위를 옮겼다. 큰 조각만 들어가는 포켓을 작은 조각이 먼저 차지하는 경우다. 고정 벽과 고정 조각의 원단 이탈 예외를 사용하는 기능 시험이며 실제 의류 마커의 수율 수치가 아니다.
- 실제 WSjk 156개 입력을 순서대로 4개 lane으로 나눠 준비했다. `piece_overlap_mm=2.2`, 모드별 전·후 순서를 번갈아 3회씩 실행했으며 아래는 중앙값이다. 측정 중 빌드는 병행하지 않았다. 원본 full import와 lane 준비를 포함하며 배치 탐색·원본 결과 인증 시간은 포함하지 않는다.
- 형상 계수 벡터는 배치 수에 비례하며 tracker 복사/복원 시 함께 복사한다. 별도 조각 쌍 수의 제곱 크기 표는 추가하지 않았다. 여러 solver phase/Structure multi-start 사이의 단일 전역 준비 공유까지 구현한 것은 아니다.
- 공동 Hole Fill은 제한된 후보 생성·기하 검사를 추가하므로 비용이 0은 아니다. raster 경로는 공동 시도가 실패하거나 이동하지 않으면 기존 grid를 재사용하고, 성공했을 때만 재구성한다. NFP 경로는 공동 후보를 위해 별도 grid 준비가 생긴다. 전체 마커의 제한시간별 수율·CPU·메모리를 비교한 결과는 아직 없다.

| 단순화 | 4개 lane 준비: 반복 import → 공유 | full+lane 총 준비: 전 → 후 | 형상 확인 |
|---|---:|---:|---|
| 면적 비율 0.01 | 1,845.759ms → 0.019ms | 3,389.980ms → 1,537.138ms | CD 정점 합 2,272, 면적 동일 |
| 최대 이탈 0.5mm | 5,633.986ms → 0.029ms | 10,852.576ms → 5,476.776ms | CD 정점 합 5,280, 면적 동일 |

이 비교에서 총 **형상 준비**는 각각 약 54.7%, 49.5% 줄었다. 전체 Init·네스팅 속도가 그만큼 빨라졌다는 뜻은 아니다. 공유 후 lane 준비 자체는 수십 마이크로초로 짧으므로 큰 배수보다 full import를 포함한 시간을 판단 기준으로 삼는다. 별도 정확성 시험에서 면적/mm 모드의 전체 CD 정점·변환·strip을 독립 import와 대조했다.

입력: `C:/Caplanet/AiWorks/InputData/여성WSjk_비교용-G_New_SwarmV1.json`, SHA256 `97BF8FF3453FF99674ACF369B54F3AA4CDABFB8F99C4CAC97716BBBAB0F0E738`. 재현 예제는 `swarm-core/examples/priorities_preparation_bench.rs`, 원시 측정과 중앙값은 `target/all-phase-priorities-validation-20260909/{lane-preparation.jsonl,lane-summary.json}`이다.

## 최종 DLL과 WPF 빌드

```powershell
cargo rustc --release --lib --crate-type cdylib --features "simd capmods adaptive structure-first jni" -v -- -C lto=fat
dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj -c Release -p:Platform=x64 --no-restore --nologo -v minimal
```

최종 DLL 컴파일 명령에서 `opt-level=3`, `lto=fat`를 확인했다. `codegen-units=1`은 설정하지 않았다. Rust의 경고 1개는 MSVC가 import library를 생성했다는 링커 출력이며, WPF는 경고 0·오류 0이다.

**빌드 확인 중 발견한 점:** 현재 Cargo의 `rlib + cdylib + staticlib` 동시 출력 명령에는 profile의 fat 설정만으로 실제 DLL의 `-C lto`가 전달되지 않았다. 세 형식에 일괄 강제하면 rlib 때문에 LTO 오류가 발생하므로, 최종 배포용 DLL만 위 명령처럼 `--crate-type cdylib`로 빌드했다. 일반 `cargo build --release --lib` 결과가 같다고 가정하면 안 된다. 소스의 crate-type 목록은 기존 테스트·정적 라이브러리 사용을 위해 유지했다. 이 발견은 과거 DLL까지 다시 분석한 결과가 아니다.

- native DLL: `target/release/capswarmcore_100.dll`
- WPF: `csharp/CAPlanetSwarmSample.WPF/bin/x64/Release/net10.0-windows10.0.19041.0/CAPlanetSwarmSample.WPF.exe`
- 두 폴더 DLL SHA256: `7A2378742FDD7922F775D5FADA17D74875A52878E0E014D2900FBF805492A6DD`
- 실제 WPF 폴더 DLL 로딩: ABI 15, config 776bytes, Structure config 116bytes, 기능 플래그 31, 제품 버전 1.0.0, 관리 코드가 요구하는 export 33개 확인.

검증 로그·산출물 기록은 `target/all-phase-priorities-validation-20260909`에 있다. 테스트 통과는 실제 WPF 화면의 수율·반응 시간을 확인했다는 뜻은 아니며, 이번에는 UI를 실행하지 않았다.

## 백업

- 전체 우선순위 작업 시작 전: `target/all-phase-priorities-before-20260909-173619` (파일별 SHA256 및 당시 Release DLL).
- shrink 롤백 직전: `target/shrink-control-rollback-before-20260909-181209`.
- shrink 복원 기준: `target/adaptive-control-implementation/before/swarm-core/src/optimizer/explore_mods.rs`.

실행 중인 외부 WPF/UTTU 프로그램 폴더에 DLL을 배포하거나 프로세스를 종료하지 않았다. Git commit/push는 이번 요청 범위에 포함하지 않았다.
