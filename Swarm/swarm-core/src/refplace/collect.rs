//! 모듈 4·5 — 후보 수집(3모드)과 정렬·분배.
//!
//! 설계 §모듈 4 의 표를 그대로 옮긴다.
//!
//! ```text
//!   모드            동작                                            정렬 키
//!   표면 점수 모드   선행 참조 대상이 미배치면 부착 예약,             표면 점수
//!                   아니면 독립 배치 후보
//!   배치 필터 모드   배치 완료된 피스만 후보                          피스 정렬 키
//!   표면 순서 모드   표면 순서대로 미처리 표면의 피스 후보            피스 정렬 키
//! ```
//!
//! §모듈 5:
//!
//! ```text
//!   키 내림차순 정렬
//!   후보 8개 미만 = 직렬 처리 / 8개 이상 = 워커 라운드로빈 병렬
//! ```

use crate::refplace::params::PARALLEL_THRESHOLD;
use crate::refplace::record::{PieceRecord, PlaceState, SurfaceRecord};

// ────────────────────────────────────────────────────────────────────────────
// 3모드 (§모듈 4)
// ────────────────────────────────────────────────────────────────────────────

/// 후보 수집 모드.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollectMode {
    /// 표면 점수 모드 — 선행 참조 대상이 미배치면 **부착 예약**, 아니면 독립 배치 후보.
    ///
    /// 정렬 키는 **표면 점수**다. 배치를 처음부터 쌓아 올릴 때 쓴다.
    SurfaceScore,
    /// 배치 필터 모드 — **배치 완료된 피스만** 후보.
    ///
    /// 정렬 키는 피스 키(현재 x). 내림차순이므로 **오른쪽부터 재평가**한다 —
    /// 오른쪽 조각이 strip 폭을 정하므로 그쪽부터 손대는 것이 이득이 크다.
    PlacedFilter,
    /// 표면 순서 모드 — 표면 순서대로 **미처리 표면**의 피스 후보.
    ///
    /// 정렬 키는 피스 키. 표면 단위로 훑을 때 쓴다.
    SurfaceOrder,
}

/// 수집된 후보 하나.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Candidate {
    /// 피스 레코드 인덱스.
    pub piece: usize,
    /// 이 후보가 나온 표면.
    pub surface: usize,
    /// 정렬 키. 모드에 따라 표면 점수이거나 피스 키다.
    pub key: f64,
    /// 부착 예약인가 — 선행 참조 대상이 아직 미배치라 대기해야 하는가.
    ///
    /// 예약된 후보는 **배치 스텝이 보류 체인에 넣는다**. 여기서 걸러 버리면
    /// 2차 패스가 다시 시도할 대상이 사라진다.
    pub deferred: bool,
}

/// 수집 결과.
#[derive(Clone, Debug, Default)]
pub struct CandidateSet {
    /// 키 **내림차순**으로 정렬된 후보들 (§모듈 5).
    pub candidates: Vec<Candidate>,
}

impl CandidateSet {
    /// 직렬로 처리할지 (§모듈 5: 후보 8개 미만 = 직렬).
    pub fn is_serial(&self) -> bool {
        self.candidates.len() < PARALLEL_THRESHOLD
    }

    /// 워커 라운드로빈 분배 (§모듈 5).
    ///
    /// 워커 `w` 는 인덱스 `w, w+n, w+2n, ...` 을 받는다. **인덱스로 나누는 것이 핵심**이다 —
    /// `len()/n` 으로 청크를 자르면 같은 입력이 워커 수에 따라 다르게 갈리고, 그러면
    /// 축약 순서가 달라져 결과가 머신마다 바뀐다. 이 저장소가 병렬 결정론에서 이미
    /// 확인한 함정이라 여기서 되풀이하지 않는다.
    pub fn round_robin(&self, workers: usize) -> Vec<Vec<usize>> {
        let n = workers.max(1);
        let mut lanes = vec![Vec::new(); n];
        for i in 0..self.candidates.len() {
            lanes[i % n].push(i);
        }
        lanes
    }

    /// 부착 예약이 아닌 후보만.
    pub fn ready(&self) -> impl Iterator<Item = &Candidate> {
        self.candidates.iter().filter(|c| !c.deferred)
    }

    /// 부착 예약된 후보만 — 보류 체인에 들어갈 것들.
    pub fn deferred(&self) -> impl Iterator<Item = &Candidate> {
        self.candidates.iter().filter(|c| c.deferred)
    }
}

/// 후보를 수집하고 키 내림차순으로 정렬한다 (§모듈 4·5).
///
/// `surface_order` 는 [`crate::refplace::chain::AttachChain::surface_order`] 다 —
/// 표면 키 내림차순이다.
///
/// # 정렬의 결정론
///
/// 키가 동률이면 **피스 인덱스 오름차순**으로 가른다. 부동소수 키만으로 정렬하면
/// 동률 구간의 순서가 정렬 알고리즘에 달리고, 그러면 같은 입력이 다른 배치를 낼 수
/// 있다. 인덱스는 전역 유일하므로 전순서가 보장된다.
pub fn collect(
    mode: CollectMode,
    pieces: &[PieceRecord],
    surfaces: &[SurfaceRecord],
    surface_order: &[usize],
    processed_surfaces: &[bool],
) -> CandidateSet {
    let mut candidates = Vec::new();

    for &s in surface_order {
        let Some(surface) = surfaces.get(s) else {
            continue;
        };
        match mode {
            CollectMode::SurfaceOrder => {
                // 미처리 표면만 본다.
                if processed_surfaces.get(s).copied().unwrap_or(false) {
                    continue;
                }
            }
            CollectMode::SurfaceScore | CollectMode::PlacedFilter => {}
        }

        for &p in &surface.pieces {
            let Some(piece) = pieces.get(p) else {
                continue;
            };
            match mode {
                CollectMode::SurfaceScore => {
                    // 이미 놓인 조각은 이 모드의 대상이 아니다.
                    if piece.is_placed() {
                        continue;
                    }
                    // 선행 참조 대상이 하나라도 미배치면 **부착 예약**이다.
                    let deferred = piece
                        .predecessors
                        .iter()
                        .any(|&q| pieces.get(q).map(|r| !r.is_placed()).unwrap_or(true));
                    candidates.push(Candidate {
                        piece: p,
                        surface: s,
                        key: surface.sort_key,
                        deferred,
                    });
                }
                CollectMode::PlacedFilter => {
                    if !piece.is_placed() {
                        continue;
                    }
                    candidates.push(Candidate {
                        piece: p,
                        surface: s,
                        key: piece.sort_key,
                        deferred: false,
                    });
                }
                CollectMode::SurfaceOrder => {
                    if piece.is_placed() {
                        continue;
                    }
                    candidates.push(Candidate {
                        piece: p,
                        surface: s,
                        key: piece.sort_key,
                        deferred: false,
                    });
                }
            }
        }
    }

    // §모듈 5 — 키 내림차순. 동률은 피스 인덱스로 갈라 전순서를 만든다.
    candidates.sort_by(|a, b| b.key.total_cmp(&a.key).then_with(|| a.piece.cmp(&b.piece)));

    CandidateSet { candidates }
}

/// 피스 정렬 키를 갱신한다 — 배치 시 설정, 이동 시 bbox 델타로 (§모듈 4).
///
/// 키는 **현재 x 위치 계열**이다. 배치 필터 모드가 내림차순으로 훑으므로 큰 x =
/// 오른쪽 = 먼저 재평가가 된다.
pub fn set_piece_key(piece: &mut PieceRecord, x_min: f64) {
    piece.sort_key = if x_min.is_finite() { x_min } else { 0.0 };
}

/// 이동한 만큼 키를 옮긴다. 배치 전체를 다시 훑지 않기 위한 것이다.
pub fn shift_piece_key(piece: &mut PieceRecord, dx: f64) {
    if dx.is_finite() {
        piece.sort_key += dx;
    }
}

/// 보류 체인에 등록한다 (§모듈 6).
///
/// 상태만 바꾼다 — 2차 패스([`crate::refplace`] 모듈 9)가 이 상태를 보고 재시도한다.
pub fn defer(piece: &mut PieceRecord) {
    if !piece.is_placed() {
        piece.state = PlaceState::Deferred;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::refplace::record::{PlacedPose, RotationEntry, RotationList};

    fn piece(item_id: usize) -> PieceRecord {
        PieceRecord::new(item_id, 0, RotationList::fixed())
    }

    fn placed(item_id: usize, x: f64) -> PieceRecord {
        let mut p = piece(item_id);
        p.state = PlaceState::Placed;
        p.sort_key = x;
        p.pose = Some(PlacedPose {
            rotation: RotationEntry::IDENTITY,
            extra_rad: 0.0,
            x,
            y: 0.0,
        });
        p
    }

    fn surface(pieces: Vec<usize>, key: f64) -> SurfaceRecord {
        let mut s = SurfaceRecord::new(100.0, 50.0);
        s.pieces = pieces;
        s.sort_key = key;
        s
    }

    /// 표면 점수 모드: 선행 참조 대상이 미배치면 **부착 예약**이다.
    #[test]
    fn surface_score_mode_defers_when_a_predecessor_is_unplaced() {
        let mut pieces = vec![piece(0), piece(1)];
        pieces[1].predecessors = vec![0];
        let surfaces = vec![surface(vec![0, 1], 10.0)];
        let set = collect(
            CollectMode::SurfaceScore,
            &pieces,
            &surfaces,
            &[0],
            &[false],
        );
        assert_eq!(set.candidates.len(), 2);
        let c0 = set.candidates.iter().find(|c| c.piece == 0).unwrap();
        let c1 = set.candidates.iter().find(|c| c.piece == 1).unwrap();
        assert!(!c0.deferred, "선행 참조가 없으면 독립 배치 후보다");
        assert!(c1.deferred, "선행 참조가 미배치면 부착 예약이다");
    }

    /// 선행 참조가 놓이면 예약이 풀린다.
    #[test]
    fn the_reservation_lifts_once_the_predecessor_is_placed() {
        let mut pieces = vec![placed(0, 5.0), piece(1)];
        pieces[1].predecessors = vec![0];
        let surfaces = vec![surface(vec![0, 1], 10.0)];
        let set = collect(
            CollectMode::SurfaceScore,
            &pieces,
            &surfaces,
            &[0],
            &[false],
        );
        // 놓인 0 은 이 모드의 대상이 아니다.
        assert_eq!(set.candidates.len(), 1);
        assert_eq!(set.candidates[0].piece, 1);
        assert!(!set.candidates[0].deferred);
    }

    /// 예약된 후보를 **버리지 않는다** — 버리면 2차 패스가 재시도할 대상이 사라진다.
    #[test]
    fn deferred_candidates_are_kept_not_dropped() {
        let mut pieces = vec![piece(0), piece(1)];
        pieces[1].predecessors = vec![0];
        let surfaces = vec![surface(vec![0, 1], 10.0)];
        let set = collect(
            CollectMode::SurfaceScore,
            &pieces,
            &surfaces,
            &[0],
            &[false],
        );
        assert_eq!(set.deferred().count(), 1);
        assert_eq!(set.ready().count(), 1);
    }

    /// 배치 필터 모드는 **놓인 조각만** 보고, 오른쪽부터 재평가한다.
    #[test]
    fn placed_filter_mode_walks_from_the_right() {
        let pieces = vec![placed(0, 10.0), piece(1), placed(2, 90.0), placed(3, 50.0)];
        let surfaces = vec![surface(vec![0, 1, 2, 3], 1.0)];
        let set = collect(
            CollectMode::PlacedFilter,
            &pieces,
            &surfaces,
            &[0],
            &[false],
        );
        let order: Vec<usize> = set.candidates.iter().map(|c| c.piece).collect();
        assert_eq!(order, vec![2, 3, 0], "x 내림차순 = 오른쪽부터, 미배치는 제외");
    }

    /// 표면 순서 모드는 **미처리 표면**만 본다.
    #[test]
    fn surface_order_mode_skips_processed_surfaces() {
        let pieces = vec![piece(0), piece(1)];
        let surfaces = vec![surface(vec![0], 9.0), surface(vec![1], 8.0)];
        let set = collect(
            CollectMode::SurfaceOrder,
            &pieces,
            &surfaces,
            &[0, 1],
            &[true, false],
        );
        assert_eq!(set.candidates.len(), 1);
        assert_eq!(set.candidates[0].piece, 1);
    }

    /// §모듈 5 — 후보 8개 미만이면 직렬.
    #[test]
    fn the_serial_threshold_is_eight() {
        let mk = |n: usize| CandidateSet {
            candidates: (0..n)
                .map(|i| Candidate {
                    piece: i,
                    surface: 0,
                    key: 0.0,
                    deferred: false,
                })
                .collect(),
        };
        assert!(mk(7).is_serial());
        assert!(!mk(8).is_serial(), "8개부터 병렬이다");
        assert_eq!(PARALLEL_THRESHOLD, 8);
    }

    /// 라운드로빈은 **인덱스로** 나눈다 — 청크로 자르면 워커 수가 결과를 바꾼다.
    #[test]
    fn round_robin_splits_by_index_not_by_chunk() {
        let set = CandidateSet {
            candidates: (0..7)
                .map(|i| Candidate {
                    piece: i,
                    surface: 0,
                    key: 0.0,
                    deferred: false,
                })
                .collect(),
        };
        let lanes = set.round_robin(3);
        assert_eq!(lanes[0], vec![0, 3, 6]);
        assert_eq!(lanes[1], vec![1, 4]);
        assert_eq!(lanes[2], vec![2, 5]);
        // 워커 수가 달라도 **같은 후보 집합**을 덮는다.
        let all: Vec<usize> = {
            let mut v: Vec<usize> = set.round_robin(5).into_iter().flatten().collect();
            v.sort_unstable();
            v
        };
        assert_eq!(all, (0..7).collect::<Vec<_>>());
    }

    /// 동률 키는 **피스 인덱스**로 갈라 전순서를 만든다 — 결정론의 전제다.
    #[test]
    fn ties_break_on_the_piece_index_so_the_order_is_total() {
        let pieces = vec![placed(0, 5.0), placed(1, 5.0), placed(2, 5.0)];
        let surfaces = vec![surface(vec![2, 0, 1], 1.0)];
        let set = collect(
            CollectMode::PlacedFilter,
            &pieces,
            &surfaces,
            &[0],
            &[false],
        );
        let order: Vec<usize> = set.candidates.iter().map(|c| c.piece).collect();
        assert_eq!(order, vec![0, 1, 2], "표면 안 순서가 아니라 인덱스로 갈라야 한다");
    }

    /// 피스 키는 배치 시 설정하고 이동 시 델타로 갱신한다 (§모듈 4).
    #[test]
    fn the_piece_key_tracks_x_through_moves() {
        let mut p = piece(0);
        set_piece_key(&mut p, 40.0);
        assert_eq!(p.sort_key, 40.0);
        shift_piece_key(&mut p, -15.0);
        assert_eq!(p.sort_key, 25.0);
        // 비유한 값은 키를 망가뜨리지 않는다.
        shift_piece_key(&mut p, f64::NAN);
        assert_eq!(p.sort_key, 25.0);
        set_piece_key(&mut p, f64::INFINITY);
        assert_eq!(p.sort_key, 0.0);
    }

    /// 보류 등록은 **놓인 조각을 되돌리지 않는다**.
    #[test]
    fn deferring_never_unplaces_something() {
        let mut p = placed(0, 1.0);
        defer(&mut p);
        assert_eq!(p.state, PlaceState::Placed);
        let mut q = piece(1);
        defer(&mut q);
        assert_eq!(q.state, PlaceState::Deferred);
    }
}
