# Swarm WPF Export JSON → Uttu.Nest 왕복

대상은 Swarm WPF sample UI의 **Export JSON** 버튼으로 만든 파일이다.

## 적용 내용

- `SwarmInputBuilder`의 V1 BuildAll/BuildOnce 및 V2 전체/사이즈그룹 항목 생성 경로 7곳에 `OrgID`와 `OrgTransform`을 포함한다. 기존 순차 id는 Swarm 계산용으로 유지한다.
- OrgTransform은 입력 기준 회전(도), 대칭, 버퍼, 마진, 정규화 이동, 원본 형상 해시다. 현재 배치 각도가 바뀌어도 입력 당시 좌표를 복원할 수 있다.
- WPF `SolutionJsonExportService`는 입력 스냅샷의 id와 결과 item_id를 연결해 layout.placed_items에 OrgID(Int64)와 OrgTransform을 붙인다. V1/V2 및 solution 래퍼를 지원한다.
- Uttu.Nest의 **Swarm 결과 Json 가져오기**는 입력 내보내기 바로 옆에 있으며, IsEnabled·Visibility·CanMarker 및 Test 리본 구성 조건이 같다.
- ID·형상·버퍼·마진·사용 폭·수치 범위를 전체 사전 검증한 뒤 한 번의 History 연산으로 적용한다. 현재 잠금 배치는 유지하며, 미배치 조각도 배치한다. Undo/Redo 및 반복 가져오기 무변경을 지원한다.
- 부분 결과의 미포함 조각은 유지한다. 잘못된 항목이 있으면 전체 적용을 거부한다. 별도의 충돌/strip 이탈 거부는 추가하지 않았다.

## 사용 순서

1. 업데이트된 Uttu.Nest에서 입력 JSON을 내보낸다.
2. 업데이트된 WPF sample에서 입력을 실행하고 Export JSON으로 저장한다.
3. 원본 마커의 Swarm 결과 Json 가져오기로 파일을 선택한다.

**예전 OrgTransform 없는 입력은 다시 내보내야 한다.** 직접 네이티브 GetSolutionJson 결과가 아닌 WPF Export JSON 파일이 연동 대상이다.

## 검증

| 항목 | 결과 |
|---|---|
| WPF 서비스 테스트 Release | 94/94 통과 |
| Uttu.Nest 테스트 Debug / Release | 각각 78/78 통과 |
| CAD 회귀 Debug / Release, Category!=Bench | 각각 2053/2053 통과 |
| WPF sample Release x64 빌드 | 오류 0, 기존 영역 경고 14 |
| Uttu.Nest Debug / TestRelease 빌드 | 오류 0, 경고 각각 118 / 166 |
| 신규 파일 빌드 경고 | 없음 |
| 리본 XML | 인접 배치 및 IsEnabled/Visibility 동일 |

실제 .unest를 읽어 입력을 만들고, 현재 네이티브 DLL의 **Init 결과** → 실제 WPF Export JSON 서비스 → Uttu Prepare/Apply/Undo를 실행했다. 전체 Explore/Compress/Polish 성능 시험은 아니다. 모든 원본 윤곽 정점을 결과 입력 윤곽의 변환 좌표와 비교했다.

| 마커 | 조각 | V1 최대 오차 | V2 최대 오차 |
|---|---:|---:|---:|
| OPS2_비교용-겉감 | 52 | 0.000644 mm | 0.000583 mm |
| 여성WSjk_비교용-G_New | 156 | 0.000640 mm | 0.000707 mm |

모두 Nest의 0.001 mm 정수 좌표 정밀도 안이다. 실제 리본 클릭은 수행하지 않았으며 앱 컴파일·XAML 바인딩·실제 Export 서비스와 모델 왕복을 검증했다.

증거는 target/result-json-*.log, 실제 왕복 파일과 도구는 target/result-json-markers/에 있다.
작업 전 Uttu 소스 백업은 target/result-json-uttu/before/ 및 hashes.json에 있다.

Rust 기하 엔진·ABI·균일 겹침 정책은 변경하지 않았다. 기존 균일 겹침 롤백 6개 소스 해시가 일치한다. 네이티브 DLL과 WPF 복사본 SHA256은 둘 다 F50D140A65F558103C644AF9D85D20326C4F447B8EC088C00143D980F27C883B다.
