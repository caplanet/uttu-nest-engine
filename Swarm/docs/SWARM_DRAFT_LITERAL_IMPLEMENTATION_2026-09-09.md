# Draft Init 순차 배치·면적 점수 적용 결과

## 적용한 내용

`AdaptiveAlgorithm2`의 WPF 리본에서 사용하는 Column Draft Init에 적용했다.

1. 조각을 작업 목록 순서대로 처리하고, 배치 가능한 위치를 찾은 첫 허용 회전에서 확정한다.
2. 필요한 NFP를 준비한 다음 고정/이동 ID와 양쪽 회전을 포함한 정렬 캐시에서 이진탐색한다.
3. 빈/실패 NFP는 부착 후보에서 제외한다. NFP가 현재 이동 창 전체를 막는 경우는 보류하고, 변경된 열/틈에서 다시 시도한다. 커널에 숫자형 전체 겹침 표식은 없어 창 전체 포함을 보수적으로 판정한다.
4. 위치 점수 계산과 접촉 보정을 합해 조각 시도당 최대 10,000회로 제한한다. 회전이 바뀌어도 같은 시도의 예산을 공유한다.
5. 사용자가 선택한 `avg_s = 배치면적 / 스트립수`, `score = v_max - 8 × max(avg_s)`를 적용한다. 종전 가중 효율 차이로 대체하지 않았다.
6. 최종 인증과 같은 원본 겹침 판정을 후보 및 접촉 보정에도 적용했다. 축소 도형만 통과한 결과가 마지막 원본 검사에서 실패하던 문제를 수정했다. 시간 소진 후 새 열 배치도 기존 원본의 오른쪽 끝을 피한다.

새 Config나 ABI 필드는 추가하지 않았다. WPF의 기존 Draft Init 선택 경로에 연결된다. Explore/Compress/Polish 알고리즘 변경은 이번 작업에 포함하지 않았다.

## 산식의 해석과 제한

현재 단일 원단 API를 **원단 1장 = 표면 1개, 스트립수 1**로 해석했다. 열을 별도 원단으로 세지 않는다. 이는 추가 표면 구분 답변이 없는 상태에서 사용자에게 알리고 적용한 가정이다.

- 면적은 Draft가 사용하는 실제 `shape_cd` 다각형 면적 합이다. 후보가 경계에 걸치면 실제 다각형을 잘라 계산한다. 원본 면적이나 bbox 면적으로 바꾸지 않았다.
- 개수는 확정 조각 수에 후보 1개를 더한 값이다.
- 기준면적은 후보를 포함한 CD 사용 길이 × CD 원단 높이다.
- `v_max`는 해당 회전의 한도 내 위치 제안 묶음에서 최대 `value`다. 위치의 NFP/정밀 통과 여부와 별개인 공통 기준값이다.
- 동일 점수에서는 `value`, 왼쪽, 아래쪽 순으로 선택한다.
- 지정한 4구간 가중치도 계산하고 리포트에 남긴다. 다만 확정한 최종 식에는 `w` 항이 없으므로 면적에 임의로 곱하거나 `w=0`인 후보를 제거하지 않는다.

**`avg_s`는 mm²이며 점유율이 아니다.** 같은 조각이 원단 안에 있는 위치끼리는 면적이 같아 주 점수가 동점이 된다. 따라서 이 식만으로 효율이나 속도가 향상된다고 볼 수 없다. 세부 구현 정의는 [DRAFT_COLUMN_INIT.md](DRAFT_COLUMN_INIT.md)에 정리했다.

## 실제 입력 비교

입력: `C:\Caplanet\AiWorks\InputData\여성WSjk_비교용-G_New_SwarmV1.json` (156조각).

조건: `poly_simpl_tolerance = 0.01`, `rng_seed = 42`, 힌트 제거, 기존 Draft 선택. 백업 DLL과 새 Release DLL을 각각 정확한 경로로 로드하고 C# ABI 검사를 통과시켰다. 각 조건을 별도 프로세스에서 3회 실행했다. 시간은 입력 초기화부터 솔루션 JSON 취득까지의 중앙값이며, 사용 길이는 3회 동일했다. 컴파일과 측정을 동시에 실행하지 않았다.

| 겹침 허용 | 변경 전 실제 경로 | 변경 전 길이 | 변경 후 길이 | 길이 감소 | Init 시간 전 → 후 |
|---|---|---:|---:|---:|---:|
| 0mm | Draft 성공 | 7603.042mm | 7208.149mm | 394.893mm | 1.195초 → 2.907초 |
| 2.2mm | Draft 원본 인증 실패 → LBF | 7248.3823mm | 7135.6562mm | 112.7261mm | 3.568초 → 4.854초 |

새 버전은 두 조건에서 모두 Draft가 성공했고 156조각을 반환했다. 로그의 `completion=false`, NFP 실패 0을 확인했다.

이번 입력에서는 길이가 감소했지만 시간은 늘었다. 0mm는 약 143%, 2.2mm는 약 36% 증가했다. 특히 **2.2mm의 이전 길이는 Draft 결과가 아니라 LBF 대체 결과**다. 순수 점수식 하나의 효과나 다른 마커의 성능으로 일반화하면 안 된다. Init만 비교했으며 이후 Explore/Compress/Polish의 최종 길이·수렴 시간은 측정하지 않았다.

측정 범위:

| 조건 | 최소–최대 Init 시간 |
|---|---:|
| 변경 전 0mm | 1.177–1.216초 |
| 변경 후 0mm | 2.895–3.073초 |
| 변경 전 2.2mm | 3.366–4.799초 |
| 변경 후 2.2mm | 4.725–5.790초 |

로그·실행 코드·원시 native 솔루션: `target/draft-formula-check/`.
집계: `target/draft-formula-check/summary.json`.
이 솔루션 파일은 벤치마크용 native 형식이며 WPF Export JSON의 가져오기 호환성을 별도로 검사한 것은 아니다.

## 검증과 빌드

- refplace 단위 검사: **112개 통과**. literal 산식, 실제 오목 다각형 클리핑, 큰 면적, 이진탐색 방향 구분, 실제 위치 탐색의 마지막 예산 이후 보정 차단 등을 포함한다.
- Column Draft 통합 검사: **9개 통과**, 수동 입력용 2개 제외. 기존에 실패하던 겹침 공차 검사 2개도 통과했다.
- 공개 Init API: **2개 통과**, 수동 입력용 1개 제외.
- 선택한 Draft가 Explore 시작 배치로 유지되는 검사: **1개 통과**.
- 별도 코드 리뷰의 평가 상한·원본 fallback 지적을 수정했고 재검토에서 남은 blocker는 발견되지 않았다.
- Rust Release 빌드 성공. 링커의 산출물 안내 경고 1개.
- WPF x64 Release 빌드 성공. CS8602/RXUISG0020 경고 14개, 오류 0개. 화면을 직접 조작하는 UI 검사는 수행하지 않았다.

```powershell
cargo build --release --lib --features 'simd capmods adaptive structure-first jni'
dotnet build csharp/CAPlanetSwarmSample.WPF/CAPlanetSwarmSample.WPF.csproj -c Release -p:Platform=x64 --no-restore --nologo -v minimal
```

Rust는 현 `Cargo.toml`의 `opt-level=3`, `lto="fat"`를 사용했다. `codegen-units=1`은 현재 주석 상태이며 이번에 강제로 지정하지 않았다.

실행 파일:
`csharp/CAPlanetSwarmSample.WPF/bin/x64/Release/net10.0-windows10.0.19041.0/CAPlanetSwarmSample.WPF.exe`.

네이티브 산출물과 WPF 출력 폴더 DLL의 SHA256이 일치한다:
`08E15BA0BFA51ABF319F4E56FDAB482178BB18B3B174E817649564253C7BA477`.

현재 다른 경로 `C:\Caplanet\AiWorks\2`에서 실행 중이던 샘플은 교체하거나 종료하지 않았다. 새 동작 확인은 위의 새 빌드 실행 파일로 해야 한다.

## 백업과 현재 상태

작업 전 소스·DLL 백업: `target/draft-input-formula-before-20260909-170102/`.
`manifest.json`에 백업 소스별 해시가 있으며 `original_overlap.rs`는 백업 루트에 별도로 저장했다.

백업 DLL SHA256:
`09900FD6A2D38326F0A04E19EAB00BECC64A6F46A871D1055ED0195C9231B885`.

입력 SHA256:
`97BF8FF3453FF99674ACF369B54F3AA4CDABFB8F99C4CAC97716BBBAB0F0E738`.

코드와 빌드는 새 방식이 적용된 상태로 유지했다. 자동 롤백이나 커밋·push는 하지 않았다. 롤백을 선택하면 이번 변경만 백업과 대조해 복원해야 하며, 다른 기존 작업을 포함한 전체 git 초기화는 하면 안 된다.
