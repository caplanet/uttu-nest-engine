# Uttu.Nest / Swarm 입력 OrgID

## 입력 계약

각 항목에 기존 Swarm `id`와 별도로 **`OrgID`**를 추가했다.

```json
{
  "id": 0,
  "OrgID": 123456789,
  "demand": 1,
  "shape": { "type": "simple_polygon", "data": [[0,0],[10,0],[0,10]] }
}
```

- `id`는 기존처럼 0부터 연속인 네이티브 조각 인덱스다.
- `OrgID`는 Uttu.Nest의 실제 단위 Place/미배치 조각인 `MkPart.ID`다. Master 또는 형상 ID가 아니다.
- 속성명은 정확히 `OrgID`이며 nullable Int64 숫자다. 이전 파일에서 없거나 null이면 원본 ID가 없는 것으로 취급한다. 임의 ID를 대신 채우지 않는다.
- `placed_hint.item_id`, region/item group 목록, 결과의 `item_id`는 기존 Swarm `id`를 계속 사용한다.
- Rust 기하 엔진은 OrgID를 계산 인덱스로 사용하지 않으며 추가 메타데이터를 수용한다. 후속 작업으로 WPF sample의 Export JSON이 입력의 `id → OrgID` 대응과 `OrgTransform`을 결과에 붙이고 Uttu.Nest 가져오기에 연결한다. 네이티브 solution/callback 자체는 기존 형식이다. [WPF 결과 왕복](SWARM_WPF_RESULT_JSON_IMPORT_2026-09-08.md)을 참고한다.

## Uttu.Nest 변경

대상 checkout: `C:\Caplanet\Products\uttu-cad`.

- `SwarmInputItem`과 `SwarmInputItemBase`에 `[JsonPropertyName("OrgID")] long? OrgID`를 추가했다. 기존 snake_case naming policy의 영향을 받지 않는다.
- `SwarmInputBuilder`의 7개 생성 지점에 원본 단위 ID를 기록했다: BuildAll의 미배치·일반 힌트·힌트 생략·잠금, BuildOnce의 미배치·기존 배치, V2 공통 AddV2Item.
- 따라서 V1 자동배치/재배치/한 번 배치, V2 all_items/size_regions에 동일하게 포함된다.
- JSON 직렬화·역직렬화는 Int64를 그대로 보존한다. `long.MaxValue`와 `2^53+1` 왕복을 시험했다.
- 모델 문서는 `domains/Uttu.Nest.Models/Docs/Swarm-Input-Identity.md`, 회귀 시험은 `tests/Uttu.Nest.Tests/Models_SwarmOrgIdTests.cs`에 추가했다.
- Uttu.Nest의 기존 다른 변경 파일과 네이티브 DLL/ABI는 교체하지 않았다.

## Swarm 변경

대상 checkout: `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`.

- WPF `SwarmInputPlaceItem.OrgID`와 V1/V2 파싱을 추가했다. 잘못된 타입/Int64 초과는 읽기 단계에서 명확하게 거부한다.
- 기존 `SwarmIOService`는 입력 JSON 문자열을 `JsonSnapshot`으로 그대로 저장하고 네이티브로 전달하므로 OrgID가 탈락하지 않는다.
- Rust 네이티브에는 이미 추가 입력 속성을 수용하는 경로가 있다. 알고리즘이나 ABI를 불필요하게 변경하지 않고 V1/V2 실제 FFI 입력 시험을 추가했다. OrgID 유무에 따른 순차 ID 및 고정 배치 좌표가 일치함을 확인했다.
- 이전에 복원한 균일 `piece_overlap_mm` 원본 검사·보정과 PLT 내보내기는 유지했다.

## 확인한 결과

- 수정 전 Uttu OrgID 회귀 시험: 8건 중 누락 관련 7건 실패, 구형 입력 1건 통과. 수정 후 Debug 8건 모두 통과.
- Uttu.Nest Release 시험 **67건 통과**, 실제 Uttu.Nest 앱 Release 빌드 성공(오류 0, 경고 116). 변경 전에도 Nest 모델 빌드 경고가 존재했으며 이번 수정 파일에서 신규 경고는 발견하지 못했다.
- Swarm WPF 시험 **89건 통과**, Release x64 빌드 성공(오류 0, 기존 경고 14).
- Rust Release V1/V2 입력 호환 시험 **2건 통과**. `long.MaxValue`, `2^53+1` OrgID를 넣어도 네이티브 조각 번호와 잠금 배치가 동일했다.
- 실제 첨부 마커를 수정된 Uttu 모델/빌더로 읽어 **OPS2 52개**, **WSjk 156개** 각각의 V1/V2를 생성했다. 모든 `OrgID`가 해당 실제 모델 단위의 `ID`와 일치하고, `id`는 0부터 순차임을 확인했다. 원본 `.unest`는 변경하지 않았다.
- 저장소 공통 CAD 게이트는 Debug **2,053건**, Release **2,053건** 통과했다. 변경 전 시작한 공통 게이트의 Debug 실행은 10분 29초, Release는 3분 15초 걸렸다. 별도의 수정 후 공통 스위트 재실행은 하지 않았으며, 실제 변경 대상인 Nest는 수정 전 실패/수정 후 Debug 및 Release 통과를 확인했다. 해당 공통 CAD 참조 프로젝트에는 소스 변경을 하지 않았다.

실제 생성 예시:

- [OPS2 V1](../target/orgid-markers/OPS2-v1.json) / [OPS2 V2](../target/orgid-markers/OPS2-v2.json)
- [WSjk V1](../target/orgid-markers/WSjk-v1.json) / [WSjk V2](../target/orgid-markers/WSjk-v2.json)

로그: `target/orgid-uttu-regression-before.log`, `target/orgid-uttu-regression-after-debug.log`, `target/orgid-uttu-nest-release-tests.log`, `target/orgid-uttu-app-build.log`, `target/orgid-wpf-tests.log`, `target/orgid-wpf-build.log`, `target/orgid-rust-input-tests.log`. 수정 전 Uttu 소스는 `target/orgid-uttu/before`에 보관했다.
