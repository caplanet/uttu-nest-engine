# UTTU.Nest 마커 실행 제한 점검 및 WPF PLT 내보내기

검사일: 2026-09-08. UTTU.Nest 분석 대상은 `C:\Caplanet\Products\uttu-cad`의 현재 소스이며, 구현 대상은 `C:\Caplanet\AiWorks\AdaptiveAlgorithm2`의 WPF sample이다. 실행 중인 별도 UTTU.Nest 배포본의 버전까지 확인한 결과는 아니다.

## 결론

같은 UTTU.Nest 프로세스에서 일반적인 자동배치·사이즈그룹 배치·재배치는 공통 잠금을 공유한다. 다른 마커로 전환하거나 Stop에서 마커의 `IsBusy`가 false가 되어도 공통 잠금이 해제되지는 않는다. 현재 소스에서 이러한 조작만으로 두 Swarm 실행이 동시에 시작되는 경로는 발견하지 못했다.

다만 다음 범위는 구분해야 한다.

1. NFP 엔진은 결과 적용과 마지막 `CompactMarker`를 UI 스케줄러에 예약하고, 이 작업의 완료를 명시적으로 기다리지 않는다. 따라서 **후처리 완료까지 보호한다는 보장이 없다**. 큐 실행이 지연되는 스케줄에서는 공통 잠금 해제 후 이전 마커의 후처리가 남을 수 있다. 실제 UTTU.Nest UI에서 이 타이밍을 재현한 결과는 아니다.
2. 공통 잠금은 한 프로세스의 `MainViewModel` 소유다. UTTU.Nest를 별도 프로세스로 여러 번 실행한 경우까지 상호 배제를 제공하지 않는다.
3. 문서 닫기 등 일부 다른 UI 기능은 공통 잠금 대신 마커별 `IsBusy`를 확인한다. Stop 직후의 문서 닫기와 늦은 후처리는 별도 보완 대상으로 볼 수 있지만, 문서 닫기 자체에서 공통 잠금을 해제하는 코드는 발견하지 못했다.

UTTU.Nest에는 검토만 수행했다. 보완하려면 NFP 결과 적용·압축을 await 가능한 UI 작업으로 연결하고, 모두 완료된 뒤 `BusyIndicator`를 해제하는 방식이 적절하다. 문서 닫기는 해당 마커의 실행·후처리가 끝났는지도 확인해야 한다.

## 소스 근거

| 경로 | 확인 내용 |
|---|---|
| [NestingActivityTracker.cs:27](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/Nesting/NestingActivityTracker.cs:27) | `Interlocked.CompareExchange`로 중복 Enter 차단. Exit까지 상태 유지 |
| [MainViewModel.cs:72](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/MainViewModel.cs:72) | 마커들이 공유하는 tracker. 94/101행의 CanStart/CanRelocation은 이 상태 참조 |
| [NestingViewModel.cs:45](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/Nesting/NestingViewModel.cs:45) | Nester 명령도 공통 상태 확인. BeginBusy/EndBusy가 Enter/Exit 담당 |
| [BusyIndicator.cs:17](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/BusyIndicator.cs:17) | Dispose 중복 호출 방지 후 EndBusy 실행 |
| [NestingViewModel.cs:192](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/Nesting/NestingViewModel.cs:192) | Stop은 취소/네이티브 Stop 및 로컬 상태 변경. 공통 Exit를 직접 호출하지 않음 |
| [NestingViewModel.cs:423](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/Nesting/NestingViewModel.cs:423) | NFP CompactMarker 예약 후 using 종료. 491/590행의 결과 적용 역시 예약 방식 |
| [NestingViewModel_Swarm.cs:760](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/Nesting/NestingViewModel_Swarm.cs:760) | 일반 Swarm 경로는 phase tasks 완료, Free, 최종 UI 반영 이후 indicator 정리 |
| [NestingViewModel_Swarm.cs:907](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/Nesting/NestingViewModel_Swarm.cs:907) | 사이즈그룹 경로 using 수명이 결과 소비 및 finally의 Free 완료까지 유지 |
| [WorkspaceViewModel.cs:149](C:/Caplanet/Products/uttu-cad/apps/Uttu.Nest/ViewModels/Workspace/WorkspaceViewModel.cs:149) | 문서 닫기는 Nester.IsBusy 확인 |

실제 tracker 소스를 연결한 실행 시험에서 선행 작업이 점유한 동안 후속 Enter 100건을 병렬 호출하여 100건 모두 거부됨을 확인했다. 해제 후 재진입도 통과했다. 큐 후처리 시험은 `예약 → Exit → 다음 Enter → 예약 작업 실행` 순서를 구성한 모델 시험이며, 실제 Dispatcher의 사용자 조작 재현과는 구분한다.

증거: [marker-lock-probe.log](../target/marker-lock-probe.log), [시험 코드](../target/marker-plt-review/Program.cs).

## 첨부 마커 확인

원본 파일은 변경하지 않고 `.unest` 본문을 읽었다. 기존 UTTU.Nest Debug DLL의 본문 디코더를 사용했다.

| 파일 | 원단 폭 | Parts 항목 | 저장된 Places | 잠긴 Places | 저장된 SizeGroups |
|---|---:|---:|---:|---:|---:|
| OPS2_비교용-겉감.unest | 1,524 mm | 52 | 52 | 0 | 0 |
| 여성WSjk_비교용-G_New.unest | 1,524 mm | 156 | 0 | 0 | 0 |

WSjk는 처음 열었을 때 저장된 배치가 없으므로 재배치 비활성화가 정상이다. 양쪽 파일 모두 저장된 사이즈그룹이 없어, 실제 그룹 실행 재현에는 UI에서 그룹을 구성해야 한다. 두 마커를 동시에 열고 세 명령과 Stop/닫기를 조합하는 전체 UI 회귀 시험은 수행하지 않았다.

추가로 기존 DLL을 통해 모델 전체를 로드하여 Swarm 입력을 새로 생성하려는 시도는 ReactiveUI 앱 초기화 의존성 때문에 완료하지 않았다. 마커 수치는 모델 재생성 결과가 아닌 디코딩된 저장 데이터에서 확인했다.

## 구현: 실제 PLT만 내보내기

WPF sample 리본 **Output → Export PLT**에 추가했다. 결과가 있고 현재 작업이 실행 중이 아닐 때 사용할 수 있다. 기존 결과 폴더에 `{마커명}_output_{시간}.plt`를 저장하고 완료 경로를 표시한다.

- JSON 추가 저장 없이 실제 ASCII HPGL 파일 한 개를 생성한다.
- Swarm **입력 JSON에 저장된 윤곽**에 결과의 회전(도)·이동(mm)을 적용한다. 원래 CAD 도형이 입력 전에 단순화되었다면 그 이전 CAD 곡선을 복구하는 기능은 아니다.
- 첨부 PLT 및 UTTU HPGL 작성 코드와 동일한 40 plotter units/mm를 사용한다. 첨부 테두리 높이 `(61802 - 842) / 40 = 1524 mm`도 일치한다.
- 조각마다 독립된 닫힌 경로와 원단 테두리를 출력한다. 단위 변경, Y축 반전, 좌표 재중심화는 하지 않는다.
- 잠금 조각을 포함해 음수/원단 밖의 결과 좌표도 그대로 출력한다. 새 충돌 검사로 결과를 거부하지 않는다.
- V1/V2 입력, 일반/래핑된 solution, 반복 item 배치를 지원한다.
- 누락 ID·빈 배치·비정상 좌표·PLT 정수 범위 초과를 검사한다. 전체 변환 성공 후 임시 파일을 이동하며 기존 파일을 덮어쓰지 않는다.
- 비동기 저장 시작 전에 마커·입력·결과를 함께 고정하여, 저장 중 탭을 전환해도 다른 마커 데이터와 섞이지 않는다.

**범위:** 첨부 PLT에 존재하더라도 Swarm 입력에 없는 내부선·문자·기타 CAD 엔티티는 생성하지 않는다. 윤곽과 원단 테두리의 HPGL 출력이다. 실제 플로터 출력이나 UTTU.Nest 재수입 화면은 확인하지 않았다.

변경 파일:

- [PltExportService.cs](../csharp/CAPlanetSwarmSample.WPF/Services/PltExportService.cs)
- [MainWindow.xaml](../csharp/CAPlanetSwarmSample.WPF/MainWindow.xaml)
- [MainWindowViewModel.cs](../csharp/CAPlanetSwarmSample.WPF/MainWindowViewModel.cs)
- [PltExportServiceTests.cs](../csharp/CAPlanetSwarmSample.WPF.Tests/PltExportServiceTests.cs) 및 테스트 프로젝트 연결 항목

## 검증 결과

- WPF 테스트 **81/81 통과**: 신규 PLT 시험 6건 포함. 좌표 변환, V2/반복 배치, 음수 좌표, 문화권, 잘못된 입력, 파일 보존을 검사했다.
- WPF **Release x64 빌드 성공**, 경고 14개 / 오류 0개. Rust는 이번 변경에 포함하지 않았다.
- 기존 실제 `여성WSjk_비교용-G_New_SwarmV1.json` 입력과 `여성WSjk_비교용-G_New_SwarmV1_output_20260908_130417.json` 결과를 이용해 PLT를 생성했다. 이는 새로운 최적화 실행 결과가 아니라 기존 Swarm 결과의 내보내기 검증이다.
- 독립 Python 검사로 **156개 배치 + 테두리 1개 = 157개 닫힌 경로**, 모든 좌표 일치를 확인했다. 축별 반올림 오차 최대 **0.0125 mm**, 파일 크기 **553,625 bytes**.
- 복사된 네이티브 DLL SHA256은 기존 `47687fb55a2680d74e1f0105db04ec605e5cb960b8e670dab31cfa6f8021716a` 그대로다.

시험 명령:

```powershell
dotnet test csharp\CAPlanetSwarmSample.WPF.Tests\CAPlanetSwarmSample.WPF.Tests.csproj -c Release --no-restore
dotnet build csharp\CAPlanetSwarmSample.WPF\CAPlanetSwarmSample.WPF.csproj -c Release -p:Platform=x64 --no-restore
python target/marker-plt-review/verify_export.py
```

산출물: [실제 출력 PLT](../target/marker-plt-review/WSjk_export_validation.plt), [좌표 검증 결과](../target/marker-plt-review/validation.json), [테스트 로그](../target/plt-export-tests.log), [빌드 로그](../target/plt-export-wpf-build.log).
