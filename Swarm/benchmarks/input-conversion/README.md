# Swarm 구조체 입력과 JSON 변환 비교

기존 `swarm_init` / `swarm_init_from_json`는 유지한다. 새 C ABI는 V2 입력을
문자열 없이 전달한다. Uttu 애플리케이션의 입력 경로를 자동으로 변경하지 않는다.

## API

선언: `include/swarm.h`, 구현: `src/ffi_input.rs`.

| 함수 | 동작 |
|---|---|
| `swarm_input_from_json(json)` | V2 JSON → 소유권을 가진 검증된 입력. 형상 import 전 반환 |
| `swarm_input_from_struct(input)` | C 구조체/배열 → 같은 검증된 입력. 형상 import 전 반환 |
| `swarm_input_free(input)` | 사용하지 않은 준비 입력 해제. NULL 허용 |
| `swarm_init_from_input(input, config)` | 준비 입력을 **성공/실패와 관계없이 소비**하고 일반 솔버 핸들 생성 |
| `swarm_init_from_struct(input, config)` | 구조체 변환과 솔버 초기화를 한 번에 수행 |

준비 입력 `SwarmInput*`와 솔버 핸들은 서로 다른 타입이다. 솔버는 기존 `swarm_free`로
해제한다. 초기화에 넘긴 준비 입력을 다시 사용하거나 해제하지 않는다. 실패 시 NULL과
기존 스레드별 `swarm_get_last_error_code` / `swarm_get_last_error_message`를 사용한다.

`CSwarmInput.abi_version=1`, `struct_size=sizeof(CSwarmInput)`를 설정한다. 이 버전은
기존 설정 구조체 ABI와 독립적이다. 모든 포인터는 호출 동안 유효해야 하고, 길이는
정확해야 한다. UTF-8 문자열은 바이트 길이를 지정하며 NUL 종료가 필요 없다.
배열 길이가 0일 때 NULL을 허용한다. 입력 변환은 버퍼를 복사하므로 호출 종료 후
호출자의 버퍼와 pin을 해제할 수 있다. 임의의 잘못된 주소의 유효성까지 검사할 수는 없다.

지원 범위:

- V2 `all_items`, `size_regions`, 배치 힌트, place group, 영역 및 간격.
- V2와 동일하게 item ID는 0부터 연속, 각 item의 demand는 1. 반복 수량은 개별 item으로 전개한다.
- `shape_kind=0`: rectangle, `1`: simple polygon. 구멍/멀티폴리곤은 이 ABI 버전에서 거부한다.
- `rotation_kind=0`: 명시적인 discrete 각도 배열, `1`: stepped, `2`: continuous.
- 각도와 반사 축은 도 단위. 힌트의 `rot_angle_in_rad`는 기존과 같이 라디안이다.
- `min_quality=-1`: 생략. `has_hint=0/1`. 영역 kind는 기존 V2 문자열을 사용한다.
- 입력 계약 검증과 그룹 동결 전파를 JSON 경로와 공유한다. 기하 유효성 검사·형상 준비는
  기존처럼 솔버 초기화 단계에서 수행된다.
- 준비용 JSON API는 V2 전용이다. 기존 초기화 API의 Legacy JSON 지원은 그대로 유지된다.
- 생성된 V2 솔버는 기존 V2 단계 규약을 따른다. Legacy 전용 `swarm_init_phase`는 지원하지 않는다.

## 측정 구간

모든 구간의 종료점은 네이티브 `ValidatedInput` 및 그 소유 핸들이 완성되는 시점이다.
형상 import, 단순화, NFP, 초기 배치, 최적화 단계와 반환 입력의 해제 시간은 포함하지 않는다.

| 측정명 | 시작점 및 포함 비용 |
|---|---|
| `native_json_pretty/compact` | 이미 준비한 UTF-8 JSON → 문법·숫자 파싱, 중간 객체, 입력 변환, 검증 |
| `native_struct` | 이미 준비한 C descriptor → 배열 복사, 입력 변환, 검증 |
| `csharp_json_pretty/compact` | 공통 C# float 좌표 객체 → JSON 문자열 직렬화, UTF-8 마샬링, 네이티브 변환·검증 |
| `csharp_struct` | 같은 객체 → 연속 정점/회전 배열과 descriptor 구성·복사·pinning, 네이티브 변환·검증 |

Uttu의 현재 JSON 옵션과 같은 `WriteIndented=true`, `SnakeCaseLower`, Unicode 허용을
기본 비교로 삼고, 공백 제거만으로 얻는 차이도 분리하기 위해 compact를 함께 측정한다.
벤치마크 DTO는 실제 Uttu 프로젝트를 참조하지 않는 독립 모델이며 좌표는 양쪽 모두 f32이다.
CAD 도형을 이 공통 객체로 만드는 비용과 파일 읽기는 제외한다.

실제 저장소의 shirts, swim, trousers, wsjk156 입력을 읽고 수량을 V2 개별 item으로 전개한다.
성능 fixture는 `all_items`, simple polygon, 기존 allowed_orientations를 사용한다.
그룹/힌트/반사/stepped 지원은 Rust 계약 테스트에서 검사하며, 해당 조합의 성능 수치는 아니다.

각 실행은 경로별 워밍업 12회 후 81회 측정한다. 경로 순서를 매 라운드 순환시켜 순서 편향을
줄인다. 관리 메모리 수치는 해당 스레드의 할당 바이트이며 peak 사용량이나 네이티브 할당량이 아니다.
각 네이티브 호출의 실패는 즉시 중단한다. 측정 전 C# P/Invoke로 JSON/구조체 입력을 실제
초기 배치까지 실행해 배치·폭·밀도가 같은지도 확인한다. 이 검사는 측정 구간 밖에서 수행한다.

## 재현 (PowerShell, 저장소 루트)

```powershell
cargo build --release --lib --features "simd,capmods,adaptive,structure-first,jni" --offline
dotnet build benchmarks/input-conversion/InputConversion.csproj -c Release
$env:DOTNET_TieredCompilation = '0'
$root = (Get-Location).Path
dotnet benchmarks/input-conversion/bin/Release/net10.0/InputConversion.dll `
  "$root/target/release/cpswcore.dll" "$root/target/input-conversion-a.json" `
  data/input/shirts.json data/input/swim.json data/input/trousers.json data/input/wsjk156.json
# 별도 프로세스로 같은 명령을 반복해 input-conversion-b.json에 저장한다.
& benchmarks/input-conversion/Summarize.ps1 target/input-conversion-summary.json `
  target/input-conversion-a.json target/input-conversion-b.json
```

DLL 로드 및 서로 다른 프로세스의 JIT 초기화는 측정 밖이다. 빌드·테스트와 성능 측정을 동시에
실행하지 않는다. 결과 JSON에는 DLL/입력 파일 SHA-256, 런타임, 모든 표본 및 관리 할당량을 남긴다.

검증 명령:

```powershell
cargo test --offline --test struct_input --test grouped_ffi -- --test-threads=1
cargo test --offline --lib -- --test-threads=1
```

외부 입력의 좌표·회전 동일성은 원본을 저장소로 복사하지 않고 검사할 수 있다.
벤치마크 로더와 마찬가지로 Legacy simple_polygon 입력을 V2 개별 item으로 전개한다.

```powershell
$env:SWARM_INPUT_PARITY_FILE = 'C:/path/to/input.json'
cargo test --offline --test struct_input benchmark_fixtures_preserve_every_coordinate_and_orientation -- --exact
Remove-Item Env:SWARM_INPUT_PARITY_FILE
```

실측 수치와 검사 결과는 로컬의 `RESULTS-*.md` 및 `results/`에 보관한다.
이 결과 보고서와 측정·로그 파일은 Git 추적에서 제외하며, 저장소에는 실행 코드와 사용법만 포함한다.

## Init 완료까지 측정

`SWARM_BENCH_MODE=init`을 지정하면 변환 반복 측정 대신 각 경로를 한 번씩 초기 배치까지
실행한다. 원본 Legacy JSON, 동일 좌표를 정규화한 V2 JSON, V2 구조체 순서다.
기본 네이티브 설정에 `rng_seed=1`을 적용한다. 파일 읽기와 공통 모델 생성은 제외하며,
C# 직렬화/구조체 배열 준비와 Init API 호출 시간을 따로 기록한다. 결과 export와 해제도 제외한다.
완료된 각 경로의 초기 배치 개수가 입력 개수와 같은지 확인하고 결과를 저장한다.
Legacy와 V2는 서로 다른 엔진 초기화 경로이므로 JSON 제거 효과는 V2끼리 비교해야 한다.

```powershell
$env:SWARM_BENCH_MODE = 'init'
$env:DOTNET_TieredCompilation = '0'
$root = (Get-Location).Path
dotnet benchmarks/input-conversion/bin/Release/net10.0/InputConversion.dll `
  "$root/target/release/cpswcore.dll" "$root/target/init-timing.json" `
  'C:/Caplanet/TestFiles/SwarmNewTestFiles/#00093_DXF 태평양-겉감_Swarm.json'
Remove-Item Env:SWARM_BENCH_MODE
```

한 경로만 실행하려면 `SWARM_INIT_CASE`를 `original_legacy_json`, `v2_json_pretty`,
`v2_struct` 중 하나로 설정한다. 경로별 출력 파일을 다르게 지정하고 실행 후 환경 변수를 지운다.
Init 모드는 입력 파일 하나와 출력 파일 하나로 실행한다. 소규모 입력이나 전체 입력 워밍업은
실행하지 않으며, 단일 표본 차이를 변환 방식의 확정적인 성능 차이로 해석하지 않는다.

### Init 내부 계측 (Legacy JSON)

별도 `init-profile` Cargo 기능과 빌드 프로파일로 내부 계측을 켠다. 기존
`target/release/cpswcore.dll`은 바뀌지 않는다. 이 진단 기능은 기본 빌드에서 제외된다.
계측 세션은 `swarm_init_from_json`에 연결돼 있으며, 상위 `phase.*` 분할은 Legacy 경로용이다.
프로세스마다 Init을 한 번씩 순차 실행한다. 동시 Init의 통합 프로파일링 용도로 사용하지 않는다.

```powershell
cargo build --offline --profile init-profile --lib `
  --features 'simd,capmods,adaptive,structure-first,jni,init-profile'
$env:SWARM_BENCH_MODE = 'init'
$env:SWARM_INIT_CASE = 'original_legacy_json'
$env:DOTNET_TieredCompilation = '0'
$root = (Get-Location).Path
$env:SWARM_INIT_PROFILE_PATH = "$root/target/init-internal-profile.json"
dotnet benchmarks/input-conversion/bin/Release/net10.0/InputConversion.dll `
  "$root/target/init-profile/cpswcore.dll" "$root/target/init-profile-timing.json" `
  'C:/Caplanet/TestFiles/SwarmNewTestFiles/#00093_DXF 태평양-겉감_Swarm.json'
Remove-Item Env:SWARM_INIT_PROFILE_PATH,Env:SWARM_INIT_CASE,Env:SWARM_BENCH_MODE
```

- `phase.*`는 서로 겹치지 않는 상위 구간이다. 세션 전체와의 잔차는 구간 전환 등이다.
- `detail.*`는 상위 구간에 포함되는 세부 시간이다. 부모와 자식 시간을 더하면 중복된다.
- `detail.import_item_parallel`은 병렬 작업별 경과 시간의 합이며 전체 형상 준비의
  벽시계 시간은 `phase.geometry`다. CPU 시간으로 해석하지 않는다.
- `detail.validation_total`은 seed 검증을 포함한 정합성 검증 호출 시간이며,
  배치 중 후보 검사와 원본 외곽선 검사 등은 각 별도 항목에 기록된다.
- 내부 `total_ms`는 프로파일 파일 쓰기를 제외한다. C#의 Init 시간은 마샬링과 파일 쓰기도
  포함하므로 미세하게 다르다. 계측 오버헤드를 별도 보정한 수치는 아니다.
- 성능 측정 중 빌드나 다른 벤치마크를 병행하지 않는다. 원본 Release 실행과
  배치 개수·폭·밀도·layout 해시를 비교해 결과 변경 여부를 확인한다.

### 초기 배치 최적화 전후 비교

`Compare-InitOptimization.ps1`은 같은 Legacy JSON 파일과 seed=1/native defaults로
변경 전 → 변경 후 → 변경 후 → 변경 전 순서의 독립 프로세스 4회를 실행한다.
계측 기능을 끈 동일 Release feature 구성의 DLL을 사용한다. 변경 전 DLL은 빌드 전에
`target/init-optimization/baseline/release-cpswcore.dll`에 보관해야 한다.

```powershell
cargo build --release --lib --offline --features 'simd,capmods,adaptive,structure-first,jni'
./benchmarks/input-conversion/Compare-InitOptimization.ps1 -Prefix 'local-repeat-01'
```

스크립트는 4,000개 배치 성공 여부와 입력·설정·layout 해시, 폭, 밀도 일치를 검사한다.
기존 측정 파일을 덮어쓰지 않으므로 재측정에는 새 `-Prefix`를 지정한다.
`-BaselineDll`, `-OptimizedDll`로 보관된 DLL 경로를 명시할 수도 있다.
측정 범위는 JSON 전달부터 초기 배치가 준비된 Init handle 반환까지이며,
디스크 읽기·결과 export·handle 해제는 제외한다.
