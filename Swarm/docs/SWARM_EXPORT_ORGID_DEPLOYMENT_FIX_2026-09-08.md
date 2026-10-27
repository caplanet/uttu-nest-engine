# WPF Export JSON OrgID 누락 재현 및 복구

## 확인한 원인

사용 파일: `C:/Caplanet/SwarmInputFiles/여성WSjk_비교용-G_New_Swarm_Result.json`.
대응 입력 `여성WSjk_비교용-G_New_Swarm.json`에는 156개 모두 OrgID와 OrgTransform이 있다.
결과에는 156개 모두 두 필드가 없고 swarm_export_version도 없다.

실행 중인 WPF 두 프로세스는 `C:/Caplanet/AiWorks/net10.0-windows10.0.19041.0/`의
이전 DLL(2026-09-08 14:02:22, 1,363,456바이트)을 사용했다. 지난 작업에서 검증한
수정본 DLL은 AdaptiveAlgorithm2/csharp 하위 x64 Release 출력(15:28:29)이었다.
입력 메타데이터가 사라진 문제가 아니라 수정본이 실제 실행 폴더에 배포되지 않은 문제다.

## 작업

- WPF 버전을 0.1.0.6으로 올렸다.
- Export JSON 저장 시 OrgID/OrgTransform이 누락되면 Uttu.Nest 가져오기가 불가능하다는
  경고를 표시하고 로그에도 기록한다. 일반 Swarm 입력에 임의 OrgID를 생성하지 않는다.
- 실제 입력 스냅샷과 item_id로 누락된 정보를 복구했다. 결과 순서와 모든 transformation은
  원본과 동일함을 JSON 구조 비교로 확인했다.
- 복구 파일: `C:/Caplanet/SwarmInputFiles/여성WSjk_비교용-G_New_Swarm_Result_OrgID.json`.
  기존 결과 파일은 보존했다.
- 실제 원본 WSjk .unest로 복구 결과 156개 Prepare/Apply/Undo를 통과했다.
- WPF 테스트 95/95 통과, Release x64 빌드 오류 0(기존 경고 14).

기존 실행 폴더 배포 스크립트는 `target/result-json-repair/deploy.ps1`이다. 대상 프로세스가
종료된 경우에만 관리 코드 5개 파일을 백업 후 교체하며, 각각 해시를 검증한다.
기존 네이티브 DLL은 유지한다. 배포 결과는 `target/result-json-deployment-apply.log`로 기록한다.

증거: `target/result-json-repair.log`, `target/result-json-repair-import.log`,
`target/result-json-deployment-tests.log`, `target/result-json-deployment-build.log`.
