import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parents[2];w=root/'target/overlap-reduction-trial';out=root/'output/overlap-reduction-2_2mm-20260909';out.mkdir(parents=True,exist_ok=True)
verification=json.loads((w/'verification22.json').read_text(encoding='utf8'));a,s,b=verification['metrics'];trial=json.loads((w/'local22-summary.json').read_text(encoding='utf8'));prep=json.loads((w/'coupled-seed22-summary.json').read_text(encoding='utf8'));prior=json.loads((w/'trial90-summary.json').read_text(encoding='utf8'))
for tag,count in [('stop-before22',0),('stop-after22',1)]:
 check=json.loads((w/(tag+'-summary.json')).read_text(encoding='utf8'));assert check['accepted']==count and check['native_final_certified']
 if count==0:assert json.loads((w/(tag+'-result.json')).read_text(encoding='utf8'))==json.loads((w/'coupled-seed22.json').read_text(encoding='utf8'))
final=out/'여성WSjk_비교용-G_New_Swarm_2_2mm_OverlapReduced.json'
shutil.copy2(w/'local22-result.json',final)
for name in ['before.json','input.json','trial90-result.json','coupled-seed22.json','verification22.json','local22-summary.json','coupled-seed22-summary.json','trial90-summary.json','stop-before22-summary.json','stop-after22-summary.json','constraints.json']:
 shutil.copy2(w/name,out/name)
assert hashlib.sha256(final.read_bytes()).hexdigest()==b['sha256']
scripts=root/'scripts/overlap_reduction_trial';scripts.mkdir(parents=True,exist_ok=True)
for name in ['experiment.py','coupled_seed22.py','verify22.py','package22.py']:
 text=(w/name).read_text(encoding='utf-8-sig').replace("root=Path(__file__).resolve().parents[2];w=root/'target/overlap-reduction-trial'","root=Path(__file__).resolve().parents[2];w=root/'target/overlap-reduction-trial'")
 (scripts/name).write_text(text,encoding='utf8')
elapsed=prior['seconds_including_setup']+prep['seconds']+trial['seconds_including_setup']
report=f'''# 원본 겹침 감소 실험 — 2.2mm (2026-09-09)

제공된 3mm 결과를 출발점으로 사용해, **원단 길이를 늘리지 않고 2.2mm 기준의 native 정밀 검증을 통과하는 JSON**을 만들었다. 원본 Swarm 입력 윤곽으로 계산한 최대 겹침 깊이 추정값은 **{a['max_estimated_depth_mm']:.4f} → {b['max_estimated_depth_mm']:.4f}mm**다. 실험 코드만 추가했으며 Explore/Compress/Polish 및 WPF 실행 파일에는 이 후처리를 연결하지 않았다.

## 바로 확인할 파일

- [Uttu.Nest 가져오기용 JSON]({final.as_posix()})
- [변경 전 결과 백업]({(out/'before.json').as_posix()})
- [정밀 검증 및 수치 상세]({(out/'verification22.json').as_posix()})

WPF Sample의 Export JSON과 같은 최상위 구조를 보존했다. 156개 조각의 `item_id`, `OrgID`, `OrgTransform`, 회전 및 나머지 모든 필드를 유지하고 `transformation.translation`만 바꿨다. 실제 Uttu.Nest 화면에서 가져오기는 실행하지 않았다. Uttu.Nest와 fable 소스에도 접근하지 않았다.

## 전후 비교

| 항목 | 제공된 3mm 결과 | 2.2mm 인증 준비 배치 | 최종 후처리 |
|---|---:|---:|---:|
| 원단 길이(mm) | {a['strip_width_mm']:.4f} | {s['strip_width_mm']:.4f} | {b['strip_width_mm']:.4f} |
| 수율(%) | {a['density']*100:.5f} | {s['density']*100:.5f} | {b['density']*100:.5f} |
| 최대 겹침 깊이 추정(mm) | {a['max_estimated_depth_mm']:.4f} | {s['max_estimated_depth_mm']:.4f} | {b['max_estimated_depth_mm']:.4f} |
| 2.2mm 초과 쌍 | {a['pairs_above_threshold_bevel_erosion']['2.2']} | {s['pairs_above_threshold_bevel_erosion']['2.2']} | {b['pairs_above_threshold_bevel_erosion']['2.2']} |
| 2mm 초과 쌍 | {a['pairs_above_threshold_bevel_erosion']['2']} | {s['pairs_above_threshold_bevel_erosion']['2']} | {b['pairs_above_threshold_bevel_erosion']['2']} |
| 1.5mm 초과 쌍 | {a['pairs_above_threshold_bevel_erosion']['1.5']} | {s['pairs_above_threshold_bevel_erosion']['1.5']} | {b['pairs_above_threshold_bevel_erosion']['1.5']} |
| 겹침이 있는 전체 쌍 | {a['overlapping_pairs']} | {s['overlapping_pairs']} | {b['overlapping_pairs']} |
| 겹침 면적 합계(mm²) | {a['total_overlap_area_mm2']:.2f} | {s['total_overlap_area_mm2']:.2f} | {b['total_overlap_area_mm2']:.2f} |
| 최초 좌표 대비 최대 이동(mm) | 0 | {s['max_translation_mm']:.4f} | {b['max_translation_mm']:.4f} |

면적 합계는 쌍별 교차 면적의 합이므로 삼중 겹침은 중복 집계된다. 깊이는 교차 영역의 최대 내접원 지름을 이용한 추정값이다. 최종 독립 재검산의 탐색 허용오차는 0.0001mm이며, 수학적으로 오차 없는 침투 깊이라는 뜻은 아니다. 표의 기준 초과 쌍은 별도로 교차 영역을 기준값의 절반만큼 bevel 침식해 남는지를 검사했다. 이와 별개로 Swarm native의 원본 검사와 탐색 도형 검사를 모두 통과했다.

원본 윤곽의 전체 X 범위는 **[{a['original_world_bounds'][0]:.6f}, {a['original_world_bounds'][2]:.6f}] → [{b['original_world_bounds'][0]:.6f}, {b['original_world_bounds'][2]:.6f}]mm**다. 선언된 strip 길이뿐 아니라 각 조각의 기존 원단 밖 돌출도 증가하지 않도록 확인했다. 원본에 이미 있던 소량의 원단 밖 돌출을 모두 0으로 없앤 결과라는 뜻은 아니다.

## 실험 방법과 채택 조건

1. 제공된 3mm/PolySimpl 0.01 결과에서 앞서 수행한 90초 겹침 감소 시험의 인증 결과를 재사용했다.
2. `piece_overlap_mm=2.2`, PolySimpl 0.01로 native 탐색 도형을 다시 만들었다. mm 단순화 모드는 사용하지 않았고 narrow-concavity 값은 현재 기본값 (0.01, 0.01)을 사용했다.
3. 맞닿는 조각들의 교차 면적 변화량을 이용해 함께 이동할 방향을 선형계획으로 제안했다. X/Y 각각 최초 위치에서 ±5mm 이내로 제한하고, strip 경계 및 원본 돌출 비증가를 제약했다. 이 준비 과정의 임시 충돌 배치는 내보내지 않는다.
4. 준비 배치의 f32 공개 좌표를 native에 다시 넣어 인증한 후에만 후처리 출발점으로 채택했다. 이 단계는 3mm에서 2.2mm로 기준을 강화하기 위한 실험용 준비 단계다.
5. 후처리는 16방향의 작은 이동을 제안하고 모든 이웃과 원본 경계를 검사했다. 최대 깊이 추정값 감소를 먼저 평가하고, 깊이가 증가하지 않는 동점 후보에서 2mm 초과 쌍 수 → 초과 교차 면적 → 전체 교차 면적 순으로 평가했다. 면적 감소만으로 최대 깊이 개선을 주장하지 않는다.
6. 채택할 후보마다 native 정밀 검증을 수행하고 인증 결과만 원자적으로 저장했다. 최종 저장 파일도 새 native 프로세스에서 다시 검사했다.

이 입력에는 잠금과 그룹이 없다고 사용자가 확인했다. 실제 156개 조각은 모두 같은 회전을 유지한다. 인증 도구에는 잠금 및 그룹 검사를 넣고 별도의 제약 변경 시험도 수행했다. 실험용 준비 스크립트는 이 데이터의 회전 0 조건을 전제로 한다. 임의 데이터에 곧바로 범용 적용하는 제품 코드가 아니다.

## 시간과 성능 해석

- 기존 3mm 후보를 얻었던 앞선 시험: **{prior['seconds_including_setup']:.2f}초**.
- 2.2mm로 전환하는 이동 제한 준비 및 인증: **{prep['seconds']:.2f}초**.
- 2.2mm 후처리: 최적화 예산 90초, 로딩·최종 검사 포함 **{trial['seconds_including_setup']:.2f}초**.
- 위 성공 경로 전체 합: **{elapsed:.2f}초**. 최종 결과를 제공된 원본에서 만들기까지의 비용을 90초만으로 계산하면 안 된다.
- 마지막 후처리의 native 검사 {trial['native_calls']}회: 합계 **{trial['native_seconds']:.2f}초**. 채택 {trial['accepted']}회, 후보 {trial['trials']:,}개.

이 시간에는 Python/Shapely 후보 생성·측정과 JSON 프로세스 통신 비용이 포함된다. Rust 제품 코드에 옮겼을 때의 지연을 확정한 벤치마크는 아니다. 컴파일, 실패한 접근의 실험 및 별도 회귀 검증 시간은 위 성공 경로 합계에 포함하지 않았다. 비교를 위한 Rust 전체 재네스팅도 실행하지 않았다. 현재는 이 데이터에서의 가능성을 확인한 결과다.

처음 시험한 기본 Separator는 인증에는 통과했지만 수 m 단위의 큰 재배치를 일으켜 제외했다. 개별 조각만 움직이는 5mm 제한 보정도 좁은 접촉 구간에서 수렴이 느려 제외하고, 이웃을 함께 움직이는 준비 방식을 사용했다.

## 검증 범위

- 실제 최종 JSON의 2.2mm native 원본 검사 및 탐색 도형 정밀 검사 통과.
- 재계산한 원본 겹침의 2.2mm 초과 쌍 0개, 최대 깊이 추정값 2.2mm 미만.
- 길이, 회전, OrgID/OrgTransform 등 좌표 외 필드 완전 일치; 원본 윤곽의 기존 경계 돌출 비증가.
- native 제약 시험 10개: 정상 결과, 원단 길이 변경, 회전 변경, 원단 밖 이동, 조각 누락·중복, 잠금 유지·위반, 그룹 유지·위반.
- 인증 직후 채택 전에 중지를 주입하면 직전 인증 파일을 유지했다. 1회 채택 직후 중지하면 그 1회 개선을 보존했다. 두 파일 모두 native 재검증 통과.
- Rust 예제 Release 빌드 성공: `cargo build -p swarm-core --example overlap_trial_native --release --features "simd capmods adaptive structure-first"`. 기존 Release 프로필의 fat LTO를 사용했다. SIMD feature를 켠 native 검사 경로이며 별도의 SIMD 성능 비교는 하지 않았다.

검사 대상 원본은 **Swarm 입력 JSON에 저장된 원본 윤곽**이다. Uttu.Nest 내부 폴리곤 또는 화면의 표시 알고리즘을 실행해 확인한 것은 아니다. 따라서 화면에 겹침 표시가 완전히 사라진다는 보장은 하지 않는다. 현재 결과도 2mm 초과 쌍이 남아 있으므로 Uttu.Nest의 표시 기준이 2mm라면 표시가 남을 수 있다.

## 재현 자료

실험 스크립트: `scripts/overlap_reduction_trial/`. Native 브리지: `swarm-core/examples/overlap_trial_native.rs`.
입력·변경 전 결과·재사용한 3mm 후보·2.2mm 준비 배치·최종 수치·중지 시험은 위 결과 폴더에 보관했다. `target/overlap-reduction-trial`에 해당 파일들을 복사한 뒤 다음 순서로 재실행할 수 있다. 시간 제한과 실행 부하 때문에 좌표가 비트 단위로 동일하게 재현된다는 보장은 없다.

```powershell
cargo build -p swarm-core --example overlap_trial_native --release --features "simd capmods adaptive structure-first"
python scripts/overlap_reduction_trial/coupled_seed22.py
python scripts/overlap_reduction_trial/experiment.py --overlap 2.2 --start coupled-seed22.json --seconds 90 --tag local22
python scripts/overlap_reduction_trial/verify22.py
```

`run_time_sec`는 원래 파일의 네스팅 시간 4224를 그대로 보존했다. 후처리 시간은 별도 summary에 기록했다. 기존 입력/결과 파일과 배포 DLL은 덮어쓰지 않았다.
'''
(root/'docs/SWARM_ORIGINAL_OVERLAP_EXPERIMENT_2_2MM_2026-09-09.md').write_text(report,encoding='utf8')
(out/'README.md').write_text(report,encoding='utf8')
print(final)
print(json.dumps({'max_depth':b['max_estimated_depth_mm'],'above2':b['pairs_above_threshold_bevel_erosion']['2'],'max_move':b['max_translation_mm'],'seconds':elapsed},indent=2))
