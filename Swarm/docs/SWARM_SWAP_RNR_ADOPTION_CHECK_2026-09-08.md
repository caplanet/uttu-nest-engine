# swap · R&R 후보의 분리·검증·채택 확인

2026-09-08, `C:\Caplanet\AiWorks\AdaptiveAlgorithm2` 현재 작업본.

사용자 요청은 임시 충돌을 포함한 탐색 후보에 후속 분리와 정밀 검증을 수행하고, 채택된 결과만 적용하는 것이다. 현재 실행 코드에 이 조건이 이미 연결되어 있어 해당 경로를 재검증했다. 이 확인을 위해 Config나 알고리즘 기본값을 변경하지 않았다.

## 적용 순서

1. 탐색용 교란 swap / R&R은 임시 탐색 배치를 만든다. 여기서 충돌이 일시 증가할 수 있다.
2. 다음 Separator 분리 결과를 얻는다. 양수 `piece_overlap_mm`를 사용하는 현재 시험 작업본에서는 원본 검사·보정도 이 경로에 연결된다.
3. 충돌 비용 0만으로 채택하지 않는다. `Separator::solution_is_certified`가 정밀 형상, 수량, 허용 회전, 잠금 좌표, 그룹 상대 좌표와 원본 겹침 허용치를 검사한다.
4. Explore / Adaptive / Escape / Reclaim은 인증에 성공한 결과만 feasible 결과로 취급하고, 단계별 개선 조건에 맞을 때 incumbent와 결과 콜백에 반영한다.
5. 분리 실패 또는 인증 실패한 후보는 최종 결과로 채택하지 않는다. 이미 인증한 결과를 유지하며, 종료 시 마지막 미분리 교란을 남기지 않도록 복원한다. Native 경계도 후보 거절 시 인증된 fallback을 유지한다.

Separator 내부 swap은 탐색용 교란 swap과 역할이 다르다. 가중 비용 감소와 비가중 국소/전체 충돌 비용 비증가 조건을 통과한 내부 이동만 유지한다. 그 내부 이동의 유지가 최종 결과 채택을 의미하지는 않는다. 최종 결과는 위 분리·정밀 인증 경계를 통과해야 한다.

잠금/잠금 겹침과 잠금 조각의 strip 이탈 예외는 유지한다. 잠금 조각의 좌표는 보정으로 바꾸지 않으며, 이동 가능한 조각과 잠금 조각의 쌍은 허용치 검사를 받는다.

## 실제 교란 회귀 결과

| 후보 | 교란 전 비용 | 교란 후 비용 | 분리 후 비용 | 판정 |
|---|---:|---:|---:|---|
| 탐색용 swap | 0 | 142.70882 | 0 | 정밀 인증 통과 |
| Random R&R | 356.81232 | 406.43317 | 1.4816208 | 후보 거절, 인증된 기존 결과 유지 |

위 숫자는 내부 충돌 비용으로 겹침 깊이(mm)가 아니다. R&R은 제한 시간 안에 항상 분리된다는 보장이 없으며, 이 시험은 실패한 후보를 채택하지 않는 동작을 확인한다. GLS 가중 비용이 좋아졌더라도 실제 충돌량을 늘린 Separator swap을 거절하는 시험도 통과했다.

## 소스와 증거

- `swarm-core/src/optimizer/lns.rs`: swap 후보 수용 조건, R&R 제안, 교란/분리 회귀 시험.
- `swarm-core/src/optimizer/separator.rs`: 원본 보정 및 `solution_is_certified` 공통 인증.
- `swarm-core/src/optimizer/explore.rs`, `explore_mods.rs`, `escape/pipeline.rs`: 분리 후 인증과 최종 incumbent 복원.
- `swarm-core/src/optimizer/reclaim.rs`: R&R 후 분리·인증 실패 시 fallback.
- `src/ffi_step.rs`: 미인증 진행 배치의 JSON 배제, 인증된 결과 캐시, 내보내기 인증.
- `target/swap-rnr-adoption-validation.log`: LNS 관련 15개 통과, 위 실제 비용 포함.
- `target/swap-rnr-adoption-native-validation.log`: Native 채택·복원·콜백·export 관련 42개 통과.
- `target/swap-rnr-adoption-original-validation.log`: 원본 허용치·잠금·그룹·취소 보정 관련 7개 통과.

총 64개 시험이 통과했다. 현재 Rust Release DLL과 WPF 실행 폴더의 DLL이 사용자 확인용 빌드와 같은 SHA256(`bdcc64585339644c04a805a76a439ef0754a8d3c1c893a25375159f0ba715de1`)임도 확인했다.

제품 실행 코드를 새로 변경한 작업은 아니다. 이전에 빌드한 Release WPF와 네이티브 DLL에 들어 있는 조건을 재검증한 것이다. 원본 검사·보정 시험 전체의 유지/롤백에 대한 사용자의 별도 결정은 이 문서로 대신하지 않는다.
