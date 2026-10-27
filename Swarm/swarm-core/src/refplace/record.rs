//! 모듈 2 — 배치 준비. 피스/표면 레코드와 그 부속 타입.
//!
//! 설계 §모듈 2 의 레코드 구조를 그대로 옮긴다. 문서가 "피스 1개 = 340바이트 고정
//! 레코드" 라고 적지만, 그건 원 엔진의 메모리 레이아웃이지 **의미론이 아니다**.
//! 여기서는 같은 **필드 집합**을 Rust 타입으로 표현한다 — 고정 크기를 흉내 내면
//! 안전성만 잃고 얻는 것이 없다.

use crate::refplace::params::FREE_ROTATION_STEP_RAD;

// ────────────────────────────────────────────────────────────────────────────
// 회전 목록 (§모듈 2, §5)
// ────────────────────────────────────────────────────────────────────────────

/// 회전 목록의 엔트리 하나 = `(x축 회전, y축 회전)` 쌍.
///
/// # 이것이 왜 "쌍" 인가
///
/// 설계 §5 가 못박는다 — 허용 회전은 **조각별 목록**이고, 엔트리는
/// `(x축 회전, y축 회전)` 쌍의 **임의 부분집합**이다. **1차원 목록이지 2차원 격자가
/// 아니다.** 즉 x 후보 4개 x y 후보 4개 = 16 조합을 전부 도는 것이 아니라, 목록에
/// 적힌 쌍만 돈다.
///
/// 축 회전은 그 축을 기준으로 뒤집는 것 — 의류 네스팅에서 말하는 **반전(flip)** 이다.
/// 그래서 이 한 타입이 회전과 반전을 함께 표현한다.
///
/// 원 엔진의 엔트리는 4바이트 `{크기(u16), x축 회전 인덱스(u8), y축 회전 인덱스(u8)}`
/// 이고 "엔트리 크기가 상태별로 다르다" 고 적혀 있다. 여기서는 그 가변 인코딩을 흉내
/// 내지 않고 의미만 남긴다.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RotationEntry {
    /// x 축 기준 뒤집기(= 상하 반전) 적용 여부.
    pub flip_x: bool,
    /// y 축 기준 뒤집기(= 좌우 반전) 적용 여부.
    pub flip_y: bool,
    /// 추가 평면 회전(밀리도, `1/1000°`).
    ///
    /// 정수로 두는 이유는 **동치 판정과 캐시 키** 때문이다. NFP 라이브러리는 회전마다
    /// 준비본을 만들므로 `0.0001°` 차이로 준비본이 두 벌 생기면 안 된다.
    /// 자유회전 스텝(`0.01°`)이 곧 `10` 밀리도라 정수로 정확히 떨어진다.
    pub millideg: i32,
}

impl RotationEntry {
    /// 회전·반전 없음.
    pub const IDENTITY: Self = Self {
        flip_x: false,
        flip_y: false,
        millideg: 0,
    };

    /// 평면 회전만 있는 엔트리.
    pub fn turn(millideg: i32) -> Self {
        Self {
            millideg,
            ..Self::IDENTITY
        }
    }

    /// 라디안 각도.
    pub fn radians(&self) -> f64 {
        f64::from(self.millideg) * std::f64::consts::PI / 180_000.0
    }

    /// 반전이 하나라도 있는가. 있으면 도형의 방향(orientation)이 뒤집힌다.
    pub fn is_mirrored(&self) -> bool {
        self.flip_x != self.flip_y
    }
}

/// 조각 하나가 쓸 수 있는 자세 목록.
///
/// 비어 있으면 [`RotationEntry::IDENTITY`] 하나만 있는 것으로 본다 — "회전 못 함" 과
/// "목록이 비었음" 을 같게 취급하는 것이 안전하다(자세를 못 찾아 배치가 통째로
/// 실패하는 것보다 낫다).
#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct RotationList {
    entries: Vec<RotationEntry>,
    /// 연속(자유) 회전 허용 여부. 켜지면 탐색이 [`FREE_ROTATION_STEP_RAD`] 스텝으로
    /// 각도를 전진시킨다(§모듈 7 자유회전).
    free: bool,
}

impl RotationList {
    /// 목록에서 만든다. 비면 항등 하나로 채운다.
    pub fn new(mut entries: Vec<RotationEntry>) -> Self {
        if entries.is_empty() {
            entries.push(RotationEntry::IDENTITY);
        }
        // **중복을 제거한다.** 같은 자세가 두 번 있으면 NFP 준비본이 두 벌 생기고,
        // 후보 열거가 그만큼 헛돈다. 순서는 보존한다 — 목록 순서가 곧 탐색 순서이고,
        // 그 순서가 결정론의 일부다.
        let mut seen = std::collections::HashSet::new();
        entries.retain(|e| seen.insert(*e));
        Self {
            entries,
            free: false,
        }
    }

    /// 자세 고정(항등 하나).
    pub fn fixed() -> Self {
        Self::new(vec![RotationEntry::IDENTITY])
    }

    /// 연속 회전을 허용한 사본.
    pub fn with_free_rotation(mut self) -> Self {
        self.free = true;
        self
    }

    /// 엔트리들.
    pub fn entries(&self) -> &[RotationEntry] {
        &self.entries
    }

    /// 연속 회전을 쓰는가.
    pub fn is_free(&self) -> bool {
        self.free
    }

    /// 자유회전일 때 한 엔트리에서 뻗을 미세 스텝 각도들(라디안).
    ///
    /// `span_rad` 만큼을 [`FREE_ROTATION_STEP_RAD`] 로 나눈다. 자유회전이 아니면 빈
    /// 반복자다 — 호출부가 분기 없이 쓸 수 있다.
    pub fn micro_steps(&self, span_rad: f64) -> impl Iterator<Item = f64> + use<> {
        let n = if self.free && span_rad > 0.0 {
            (span_rad / FREE_ROTATION_STEP_RAD).floor().max(0.0) as usize
        } else {
            0
        };
        (0..=n).map(|i| i as f64 * FREE_ROTATION_STEP_RAD).take(n + 1)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 간격 설정 (§모듈 2 연성 간격)
// ────────────────────────────────────────────────────────────────────────────

/// 두 조각의 요구 간격을 어떻게 합칠지. `[GAP TYPE]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GapType {
    /// 두 조각의 요구 간격을 **더한다**.
    #[default]
    Sum,
    /// 둘 중 **큰 쪽**을 쓴다.
    Max,
}

/// 간격을 적용할 범위. `[GAP BEHAVIOUR]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum GapBehaviour {
    /// 컨테이너 모서리에는 간격을 요구하지 않는다.
    #[default]
    IgnoreEdges,
    /// 세그먼트 길이가 [`GapSettings::smart_min_segment`] 이상인 경계에만 요구한다.
    ///
    /// 짧은 노치·톱니에까지 간격을 요구하면 맞물림이 통째로 막힌다. "스마트" 는 그것을
    /// 피하려는 규칙이다.
    Smart,
}

/// 조각 간 요구 간격 — **연성 제약**이다.
///
/// 설계 §6-6: LP 와 잔여조각 처리에서 이 요구를 **완화 가능한** 제약으로 쓴다.
/// 즉 간격을 못 지키면 배치를 포기하는 것이 아니라, 간격을 줄여서라도 넣는다.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GapSettings {
    /// 요구 간격(mm). `[GAP]`.
    pub gap_mm: f64,
    /// 합산 방식.
    pub gap_type: GapType,
    /// 적용 범위.
    pub behaviour: GapBehaviour,
    /// `Smart` 일 때 간격을 요구할 최소 세그먼트 길이(mm). `[SMART GAP CONDITION]`.
    pub smart_min_segment: f64,
    /// 수평 수축(mm). `HSHRINK`.
    pub hshrink: f64,
    /// 수직 수축(mm). `VSHRINK`.
    pub vshrink: f64,
    /// LP 가 요구 간격을 완화할 때 유지할 비율.
    ///
    /// 설계 §6-6 의 "부분 폐쇄(90%)". `1.0` 이면 완화하지 않는다.
    pub lp_partial_closure: f64,
}

impl Default for GapSettings {
    fn default() -> Self {
        Self {
            gap_mm: 0.0,
            gap_type: GapType::Sum,
            behaviour: GapBehaviour::IgnoreEdges,
            smart_min_segment: 0.0,
            hshrink: 0.0,
            vshrink: 0.0,
            lp_partial_closure: 0.9,
        }
    }
}

impl GapSettings {
    /// 두 조각 사이에 요구할 간격(mm).
    ///
    /// 지금은 모든 조각이 같은 요구 간격을 갖는다고 본다. 조각별 간격이 생기면
    /// 인자를 둘로 늘리고 [`GapType`] 대로 합치면 된다 — 그래서 합치는 규칙을
    /// 호출부가 아니라 여기 둔다.
    pub fn between(&self, a_mm: f64, b_mm: f64) -> f64 {
        match self.gap_type {
            GapType::Sum => a_mm + b_mm,
            GapType::Max => a_mm.max(b_mm),
        }
    }

    /// 이 길이의 경계 세그먼트에 간격을 요구하는가.
    pub fn applies_to_segment(&self, length_mm: f64) -> bool {
        match self.behaviour {
            GapBehaviour::IgnoreEdges => true,
            GapBehaviour::Smart => length_mm >= self.smart_min_segment,
        }
    }

    /// LP 가 완화한 뒤의 요구 간격.
    pub fn relaxed(&self) -> f64 {
        self.gap_mm * self.lp_partial_closure.clamp(0.0, 1.0)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 피스 레코드 (§모듈 2)
// ────────────────────────────────────────────────────────────────────────────

/// 배치 상태.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PlaceState {
    /// 아직 안 놓았다.
    #[default]
    Pending,
    /// 부착 대상이 미배치라 대기 중(보류 체인).
    Deferred,
    /// 놓았다.
    Placed,
    /// 놓지 못했다(잔여 조각).
    Failed,
    /// 입력에서 좌표가 고정된 조각. 시드로 쓰고 옮기지 않는다.
    Locked,
}

/// 피스 하나의 배치 레코드. 설계 §모듈 2 의 필드 집합을 그대로 갖는다.
#[derive(Clone, Debug)]
pub struct PieceRecord {
    /// 인스턴스의 아이템 id.
    pub item_id: usize,
    /// 이 레코드가 그 아이템의 몇 번째 사본인지 (demand > 1).
    pub copy_index: usize,
    /// 배치 상태.
    pub state: PlaceState,
    /// 정렬 키 — **현재 x 위치 계열**. 배치 시 설정하고 이동 시 bbox 델타로 갱신한다.
    ///
    /// 설계 §모듈 4: "배치 완료 피스 재평가 모드에서 내림차순 = 오른쪽부터 재평가".
    pub sort_key: f64,
    /// 부착 대상 피스 인덱스. `None` 이면 독립 배치.
    pub attach_to: Option<usize>,
    /// 선행 참조 리스트 — 이 피스보다 먼저 놓여야 하는 피스들.
    pub predecessors: Vec<usize>,
    /// 바인딩 쌍 — 표면 연속쌍에서 함께 묶인 상대.
    pub bound_with: Option<usize>,
    /// 재매핑 인덱스 — 정렬 후 원래 자리를 되찾기 위한 것.
    pub remap: usize,
    /// 이 조각이 쓸 수 있는 자세 목록.
    pub rotations: RotationList,
    /// 확정된 자세. `state == Placed` 일 때만 의미가 있다.
    pub pose: Option<PlacedPose>,
}

/// 확정된 배치 자세.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlacedPose {
    /// 적용한 회전·반전.
    pub rotation: RotationEntry,
    /// 자유회전으로 추가된 각도(라디안). 자유회전이 아니면 `0.0`.
    pub extra_rad: f64,
    /// 평행이동(mm).
    pub x: f64,
    pub y: f64,
}

impl PieceRecord {
    /// 새 레코드. 상태는 `Pending`, 부착 대상 없음.
    pub fn new(item_id: usize, copy_index: usize, rotations: RotationList) -> Self {
        Self {
            item_id,
            copy_index,
            state: PlaceState::Pending,
            sort_key: 0.0,
            attach_to: None,
            predecessors: Vec::new(),
            bound_with: None,
            remap: 0,
            rotations,
            pose: None,
        }
    }

    /// 놓였는가.
    pub fn is_placed(&self) -> bool {
        matches!(self.state, PlaceState::Placed | PlaceState::Locked)
    }
}

// ────────────────────────────────────────────────────────────────────────────
// 표면 레코드 (§모듈 3)
// ────────────────────────────────────────────────────────────────────────────

/// 접촉 폴리라인의 한 점과 그 연결 방향.
///
/// 설계 §모듈 7: "방향 플래그(4비트)로 상태를 관리" / "재귀 병합(방향 플래그로
/// 순방향/역방향 연결)한 접촉 경계 폴리라인".
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPoint {
    pub x: f64,
    pub y: f64,
    /// 4비트 방향 플래그. 하위 2비트가 순/역방향, 상위 2비트가 병합 상태다.
    pub flags: u8,
}

impl ContactPoint {
    /// 순방향 연결 비트.
    pub const FORWARD: u8 = 0b0001;
    /// 역방향 연결 비트.
    pub const BACKWARD: u8 = 0b0010;
    /// 이미 병합에 쓰인 점.
    pub const CONSUMED: u8 = 0b0100;
    /// 폴리라인의 끝점.
    pub const TERMINAL: u8 = 0b1000;

    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y, flags: 0 }
    }

    pub fn has(&self, bit: u8) -> bool {
        self.flags & bit != 0
    }
}

/// 표면 레코드. 설계 §모듈 3 의 필드를 그대로 갖는다.
///
/// "표면" 은 배치가 쌓이는 단위다 — 원단 한 장, 또는 마커 한 구간. 균형 점수(§4-2)가
/// 표면 사이의 형평을 보므로, 표면이 하나뿐이면 그 항은 자연히 사라진다.
#[derive(Clone, Debug)]
pub struct SurfaceRecord {
    /// 이 표면에 속한 피스 인덱스들.
    pub pieces: Vec<usize>,
    /// 표면 정렬 키 — **최대 피스 bbox 대각 계열** 값. 내림차순 = 큰 조각 우선.
    ///
    /// 설계 §모듈 4 의 산출 체인: `피스 bbox w²+h² 최대값 -> SQRT -> 고정소수화`.
    pub sort_key: f64,
    /// 접촉 폴리라인 점 목록.
    pub contour: Vec<ContactPoint>,
    /// 중첩 참조 — 다른 표면과 겹칠 때의 방향 플래그.
    pub overlap_ref: Option<(usize, u8)>,
    /// 루트 배치의 시드 좌표.
    ///
    /// 설계 §모듈 3: 기본값은 컨테이너 원점(좌하단), `LOCKED` 입력 조각은 입력 좌표.
    pub seed: (f64, f64),
    /// 이 표면의 컨테이너 크기(mm). 효율의 분모를 만든다.
    pub extent: (f64, f64),
    /// 배치 후 컨테이너 기준으로 기록해 둔 값 — §4-2 의 **기준값**.
    pub baseline: f64,
}

impl SurfaceRecord {
    /// 컨테이너 크기로 빈 표면을 만든다. 시드는 원점(좌하단).
    pub fn new(width: f64, height: f64) -> Self {
        Self {
            pieces: Vec::new(),
            sort_key: 0.0,
            contour: Vec::new(),
            overlap_ref: None,
            seed: (0.0, 0.0),
            extent: (width, height),
            baseline: 0.0,
        }
    }

    /// 기준 면적 = 현재 길이(폭) x 높이 (§4-1).
    pub fn reference_area(&self, current_width: f64) -> f64 {
        current_width.max(0.0) * self.extent.1.max(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 회전 목록은 **1차원 부분집합**이지 격자가 아니다.
    ///
    /// 격자로 오해하면 `(flip_x, flip_y)` 4조합 x 각도 N 개를 전부 돌게 되고, 준비해야
    /// 할 NFP 준비본이 몇 배로 는다. 설계 §5 가 명시적으로 아니라고 적는다.
    #[test]
    fn the_rotation_list_is_a_subset_not_a_grid() {
        let list = RotationList::new(vec![
            RotationEntry::IDENTITY,
            RotationEntry {
                flip_x: true,
                flip_y: false,
                millideg: 0,
            },
            RotationEntry {
                flip_x: false,
                flip_y: true,
                millideg: 180_000,
            },
        ]);
        // 격자였다면 2(flip_x) x 2(flip_y) x 2(각도) = 8 개가 됐을 것이다.
        assert_eq!(list.entries().len(), 3);
    }

    /// 같은 자세가 두 번 들어오면 하나로 줄인다 — 준비본이 두 벌 생기면 안 된다.
    #[test]
    fn duplicate_poses_collapse_but_order_is_preserved() {
        let list = RotationList::new(vec![
            RotationEntry::turn(90_000),
            RotationEntry::IDENTITY,
            RotationEntry::turn(90_000),
        ]);
        assert_eq!(list.entries().len(), 2);
        assert_eq!(list.entries()[0], RotationEntry::turn(90_000));
        assert_eq!(list.entries()[1], RotationEntry::IDENTITY);
    }

    /// 빈 목록은 "회전 못 함" 으로 본다 — 자세가 없어 배치가 실패하면 안 된다.
    #[test]
    fn an_empty_list_falls_back_to_identity() {
        let list = RotationList::new(Vec::new());
        assert_eq!(list.entries(), &[RotationEntry::IDENTITY]);
    }

    /// 밀리도는 자유회전 스텝(0.01°)에 정확히 떨어져야 한다.
    #[test]
    fn the_free_step_lands_on_an_exact_millidegree() {
        // 0.01도 = 10 밀리도.
        let one_step = RotationEntry::turn(10);
        assert!((one_step.radians() - FREE_ROTATION_STEP_RAD).abs() < 1.0e-15);
    }

    /// 반전이 하나면 방향이 뒤집히고, 둘이면 180도 회전과 같다.
    #[test]
    fn two_flips_are_a_turn_not_a_mirror() {
        let one = RotationEntry {
            flip_x: true,
            flip_y: false,
            millideg: 0,
        };
        let two = RotationEntry {
            flip_x: true,
            flip_y: true,
            millideg: 0,
        };
        assert!(one.is_mirrored());
        assert!(!two.is_mirrored(), "두 축을 다 뒤집으면 180도 회전이다");
    }

    /// 간격 합산 규칙이 `GAP TYPE` 대로 동작하는지.
    #[test]
    fn gap_type_decides_how_two_requirements_combine() {
        let sum = GapSettings {
            gap_type: GapType::Sum,
            ..GapSettings::default()
        };
        let max = GapSettings {
            gap_type: GapType::Max,
            ..GapSettings::default()
        };
        assert_eq!(sum.between(2.0, 3.0), 5.0);
        assert_eq!(max.between(2.0, 3.0), 3.0);
    }

    /// 스마트 간격은 짧은 세그먼트를 건너뛴다 — 노치까지 간격을 요구하면 맞물림이 막힌다.
    #[test]
    fn smart_gap_skips_short_segments() {
        let smart = GapSettings {
            behaviour: GapBehaviour::Smart,
            smart_min_segment: 5.0,
            ..GapSettings::default()
        };
        assert!(!smart.applies_to_segment(3.0));
        assert!(smart.applies_to_segment(7.0));
        // 모서리 무시 모드는 길이를 보지 않는다.
        let ignore = GapSettings::default();
        assert!(ignore.applies_to_segment(0.1));
    }

    /// 간격은 **연성**이다 — LP 가 부분 폐쇄(90%)로 완화한다.
    #[test]
    fn the_gap_requirement_is_soft() {
        let g = GapSettings {
            gap_mm: 10.0,
            ..GapSettings::default()
        };
        assert!((g.relaxed() - 9.0).abs() < 1.0e-12);
    }

    /// 자유회전이 꺼져 있으면 미세 스텝이 하나(0도)뿐이다.
    #[test]
    fn micro_steps_are_inert_unless_free_rotation_is_on() {
        let fixed = RotationList::fixed();
        assert_eq!(fixed.micro_steps(1.0).count(), 1);
        let free = RotationList::fixed().with_free_rotation();
        // 1 라디안 구간이면 0.01도 스텝이 5000개 이상 나온다.
        assert!(free.micro_steps(1.0).count() > 5_000);
    }
}
