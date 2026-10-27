# CPU 코어 고정 + 높은 우선순위 — 향후 적용 계획 (2026-10-01)

**상태: 미구현. 계획만 적어 둔다.** 사용자가 지시하면 이 문서를 기준으로 적용한다.

## 1. 목적

swarm 최적화 스레드를 지정한 코어(예: 1·2·3)에서만 돌리고, 그 코어에서 다른 프로세스보다
먼저 실행되게 한다. 기대 효과는 둘이다.

* 측정·현장 실행의 편차 감소 — 다른 프로세스가 끼어들어 탐색 처리량이 흔들리는 것을 줄인다.
* 혼합 코어 PC(예: Core Ultra 7 258V, P코어 4 + E코어 4)에서 성능 코어에만 올린다.

이 방식은 «독점» 이 아니다. 다른 프로세스가 그 코어에 **배정되는 것 자체**는 막지 못하고,
swarm 스레드가 우선순위에서 이길 뿐이다. swarm 은 CPU 를 계속 쓰는 작업이라 실효는 독점에
가깝다. OS 수준에서 배정까지 막으려면 §6 의 예약 CPU 세트가 필요하다.

## 2. 채택안 — 작업 스레드에만 적용 (B안)

프로세스 전체에 거는 방법(A안, C# 2줄)은 WPF 화면 스레드까지 같은 코어에 묶여 최적화 중
UI 가 버벅일 수 있다. 그래서 **swarm 작업 스레드와 단계 호출 스레드에만** 건다.

```csharp
// A안 (참고용, 벤치 전용 실행 파일에는 이것으로 충분하다)
var p = Process.GetCurrentProcess();
p.ProcessorAffinity = (IntPtr)0b1110;          // 코어 1,2,3
p.PriorityClass = ProcessPriorityClass.High;
```

## 3. 추가할 코드

### 3-1. 정책 모듈 (신규, 약 60줄) — `swarm-core/src/util/cpu_policy.rs`

```rust
static AFFINITY_MASK: AtomicU64 = AtomicU64::new(0);   // 0 = 끔 (기본)
static PRIORITY: AtomicI32 = AtomicI32::new(0);        // 0 보통, 1 높음, 2 가장 높음

pub fn set(mask: u64, priority: i32) -> bool { /* 범위 검사 후 저장 */ }

/// 현재 스레드에 정책을 적용한다. 꺼져 있거나 Windows 가 아니면 아무것도 안 한다.
pub fn apply_to_current_thread() {
    #[cfg(windows)]
    unsafe {
        let mask = AFFINITY_MASK.load(Relaxed);
        if mask != 0 {
            SetThreadAffinityMask(GetCurrentThread(), mask as usize);
        }
        match PRIORITY.load(Relaxed) {
            1 => { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL); }
            2 => { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST); }
            _ => {}
        }
    }
}

/// 호출 스레드용 범위 가드 — 적용하고, drop 에서 원래 선호도·우선순위로 되돌린다.
pub struct ThreadPolicyGuard { /* 이전 mask, 이전 priority */ }
```

* `swarm-core/Cargo.toml` 에 Windows 전용 의존성 `windows-sys`
  (`Win32_System_Threading`) 추가. `[target.'cfg(windows)'.dependencies]` 아래에 둔다 —
  wasm32·Linux 빌드에 영향이 없어야 한다.
* `swarm-core/src/util/mod.rs` 에 `pub mod cpu_policy;`.

### 3-2. 작업 스레드 풀에 연결 (풀 생성 지점마다 1줄)

```rust
rayon::ThreadPoolBuilder::new()
    .num_threads(n_workers)
    .start_handler(|_| crate::util::cpu_policy::apply_to_current_thread())   // 추가
    .build()
```

제품 경로의 풀 생성 지점 (2026-10-01 기준):

| 위치 | 용도 |
|---|---|
| `swarm-core/src/optimizer/separator.rs` `Separator::new` | 분리기 워커 풀 (Explore·Compress·Polish 전부) |
| `src/grouped_engine.rs` `run_scheduled_with_callbacks` | 그룹 레인 풀 |
| `src/grouped_engine.rs` structure P1 레인 풀 | structure-first 경로 |

`separator/early_report.rs`·`polish/lp.rs` 의 `ThreadPoolBuilder` 는 테스트 전용이라 대상이 아니다.

### 3-3. 단계 호출 스레드 (약 25줄)

LP 풀이(microlp)·인증·R&R 일부는 풀이 아니라 **단계를 호출한 스레드**에서 돈다. 그 스레드는
호스트(WPF) 소유이므로 `ThreadPolicyGuard` 로 적용하고 반환 시 되돌린다. 넣을 곳은
`src/ffi_step.rs` 의 `swarm_init*`, `swarm_init_phase`, `swarm_alg_phase`,
`swarm_cp_phase`, `swarm_refine_phase`, `swarm_gp_phase` 진입부
(`begin_phase` 직후).

### 3-4. 외부 API (약 15줄 + 바인딩)

```rust
/// mask: 비트 i = 논리 코어 i. 0 이면 끔. priority: 0 보통 / 1 높음 / 2 가장 높음.
/// 반환: 1 적용됨, 0 거부(범위 밖).
#[unsafe(no_mangle)]
pub extern "C" fn swarm_set_cpu_policy(affinity_mask: u64, priority: i32) -> i32
```

* `swarm_set_shrink_step` 과 같은 형태의 독립 함수다. `CSwarmConfig` 를 건드리지 않으므로
  **구조체 크기와 `SWARM_ABI_VERSION` 은 바뀌지 않는다.**
* 바인딩: `include/swarm.h` 선언, `csharp/Natives/SwarmNative.cs` 의 `DllImport` 와
  `RequiredExports` 목록, `csharp/Natives/SwarmOptimizer.cs` 래퍼, `src/jni_glue.rs`(JNI 를
  쓰는 호스트가 있으면).
* 풀은 생성 시점에 정책을 읽으므로 **Init 전에 호출**해야 그 실행에 반영된다. 문서에 명시한다.

### 3-5. WPF 설정 (선택)

* 코어 선택(체크박스 또는 마스크 입력) + 우선순위 콤보. 기본값은 «끔».
* **워커 수 ≤ 선택한 코어 수** 검사. 워커가 코어보다 많으면 고정한 코어 안에서 서로 밀어내
  오히려 느려진다. 벤치 스크립트(`scripts/full_bench/run.ps1`)가 `RAYON_NUM_THREADS` 를
  마스크와 맞추는 것과 같은 이유다.

## 4. 주의할 점

* 우선순위는 «가장 높음(THREAD_PRIORITY_HIGHEST)» 까지만. Realtime 은 그 코어에서 OS 작업까지
  굶길 수 있어 넣지 않는다.
* rayon **전역 풀**에서 도는 병렬 구간(예: `original_overlap` 의 쌍 사전 계산, 풀 밖에서 부르는
  `par_iter`)은 3-2 로 잡히지 않는다. 대부분의 작업은 분리기 풀 안에서 돌지만, 전부 고정하려면
  전역 풀을 swarm 이 먼저 초기화하면서 같은 핸들러를 걸어야 한다 — 호스트가 이미 전역 풀을
  썼다면 불가능하다. 필요해지면 그 구간을 분리기 풀의 `install` 안으로 옮기는 편이 낫다.
* 코어 번호와 P/E코어 대응은 PC 마다 다르다. 마스크를 하드코딩하지 않는다.
* 논리 코어가 64개를 넘는 PC(프로세서 그룹)는 `SetThreadAffinityMask` 로 다 표현되지 않는다.
  그 경우는 지원하지 않는다고 문서에 적고 거부한다.
* 벤치 비교 시 두 팔의 마스크·우선순위가 같아야 한다. 측정 조건으로 기록한다.

## 5. 검증

* 단위 테스트: 정책을 건 뒤 풀 스레드에서 `SetThreadAffinityMask` 를 다시 불러 반환값(이전
  마스크)이 설정값과 같은지, 가드 drop 뒤 호출 스레드의 마스크·우선순위가 원래대로인지.
* 꺼져 있을 때(`mask == 0`) 결과가 한 비트도 바뀌지 않는지 — 기존 결정론 테스트
  (`tests/nfp_parallel_determinism.rs` 등)가 그대로 통과해야 한다.
* FullBench: 같은 시드로 고정 켬/끔을 3시드씩 재서 Explore 처리량(`evals/s`)과 최종 효율의
  편차를 비교한다. 다른 부하(브라우저·빌드)가 있는 상태에서도 한 번 잰다.

## 6. 참고 — OS 수준 격리 (예약 CPU 세트)

배정 자체를 막아야 하면 Windows 의 예약 CPU 세트를 쓴다. PC 전체 설정이고 관리자 권한과
재부팅이 필요하다.

* 레지스트리 `HKLM\System\CurrentControlSet\Control\Session Manager\kernel\ReservedCpuSets`
  (비트마스크). 예약된 코어에는 OS 가 다른 스레드·인터럽트를 기본 배정하지 않는다.
* 예약 코어는 CPU Sets API 로 **명시적으로 선택한 스레드만** 쓴다. 그래서 3-1 의 적용 함수가
  `SetThreadAffinityMask` 대신 `SetThreadSelectedCpuSets`(또는 프로세스 단위
  `SetProcessDefaultCpuSets`)를 부르는 분기가 추가로 필요하다.
* Microsoft 문서상 CPU 세트는 «소프트 정책» 이라 보장은 아니다. 이 저장소에서 직접 써 본 적은
  없다 — 적용 전에 대상 PC 에서 확인할 것.

출처: [CPU Sets (Microsoft Learn)](https://learn.microsoft.com/en-us/windows/win32/procthread/cpu-sets),
[SetProcessDefaultCpuSets](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-setprocessdefaultcpusets),
[valleyofdoom/ReservedCpuSets](https://github.com/valleyofdoom/ReservedCpuSets).

## 7. 분량 추정

Rust 약 120줄(모듈 60 + 가드·연결 45 + API 15), C#·헤더 약 40줄, WPF UI 는 별도.
