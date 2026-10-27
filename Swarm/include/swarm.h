#ifndef SWARM_H
#define SWARM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define SWARM_ABI_VERSION UINT32_C(20)
#define SWARM_FEATURE_SIMD UINT32_C(0x00000001)
#define SWARM_FEATURE_CAPMODS UINT32_C(0x00000002)
#define SWARM_FEATURE_ADAPTIVE UINT32_C(0x00000004)
#define SWARM_FEATURE_STRUCTURE_FIRST UINT32_C(0x00000008)
#define SWARM_FEATURE_JNI UINT32_C(0x00000010)

/* ════════════════════════════════════════════════════════════════
 *  Configuration structs  (must match src/ffi.rs #[repr(C)] layout)
 * ════════════════════════════════════════════════════════════════ */

/** Separator configuration (shared by exploration & compression). */
typedef struct {
    int32_t iter_no_imprv_limit;
    int32_t strike_limit;
    int32_t n_workers;
    int32_t n_container_samples;
    int32_t n_focussed_samples;
    int32_t n_coord_descents;
    /* -- capmods-only fields (ignored by libraries built without the capmods feature) -- */
    int32_t swap_period;          /* swap move period in inner-loop iterations; 0 = off, negative = default (20) */
    int32_t swap_attempts;        /* swaps attempted per pass; <= 0 = default (8), clamped to 1024 */
    float   swap_area_ratio_min;  /* partner area-ratio lower bound; <= 0 = default (0.4) */
    float   swap_area_ratio_max;  /* partner area-ratio upper bound; < min = default (2.5) */
    float   left_bias_gain;       /* max left-bias strength of container samples; 0 = uniform, negative = default (2.0) */
} CSeparatorConfig;

/** Exploration phase configuration. */
typedef struct {
    float            shrink_step;
    float            time_limit_secs;
    int32_t          max_conseq_failed_attempts;   /* <= 0 = None (무제한) */
    float            solution_pool_distribution_stddev;
    float            large_item_ch_area_cutoff_percentile;
    CSeparatorConfig separator;
    /* -- adaptive-only fields (ignored by libraries built without the adaptive feature) -- */
    int32_t rnr_period;               /* ruin & recreate every N disruptions; 0 = off, negative = default (3) */
    float   rnr_band_fraction;        /* ruined vertical band width as fraction of strip width; <= 0 = default (0.18) */
    int32_t rnr_max_items;            /* max items ruined at once; <= 0 = default (40) */
    float   adaptive_shrink_gain;     /* density-adaptive shrink gain; 0 = fixed step, negative = default (4.0) */
    float   adaptive_shrink_max_mult; /* adaptive shrink max multiplier; < 1 = default (5.0) */
    int32_t osc_fail_threshold;       /* width oscillation after N consecutive fails at a width; 0 = off, negative = default (4) */
    float   osc_expand_steps;         /* oscillation expansion in base shrink_step multiples; <= 0 = default (3.0).
                                       * Effective expansion is min(shrink_step * this, headroom / 2) and headroom is
                                       * always one shrink step, so the oscillation can never pass the best feasible
                                       * width. Use CWidthReliefConfig to escape beyond it. */
    float   alns_decay;               /* ALNS operator-weight decay (0~1); out of range = default (0.85) */
    float   alns_min_weight;          /* ALNS operator-weight floor; negative = default (0.1) */
} CExplorationConfig;

/**
 * Exploration width relief (ABI v4 tail block of CSwarmConfig).
 *
 * Escapes a width the separator cannot solve by widening the strip in small
 * steps instead of retrying the same width forever. After `fail_threshold`
 * consecutive separation failures at one width the best colliding layout is
 * kept and separation is re-run at +step, +2*step, ... of the current width
 * (each expansion is inserted at the collision hot spot) until the overlaps
 * clear or `max_ratio` is reached. Shrinking then resumes from the untangled
 * layout.
 *
 * Unlike `osc_fail_threshold` this is NOT capped by the best feasible width.
 * The incumbent solution is still preserved and a wider phase result is
 * rejected at the phase boundary, so the reported result cannot get worse.
 *
 * `fail_threshold > 0` enables relief and makes `osc_fail_threshold` inert.
 * Adaptive-only; ignored by libraries built without the adaptive feature.
 */
typedef struct {
    int32_t fail_threshold; /* consecutive fails at one width before relief; 0 = off (default) */
    float   step;           /* per-round expansion as a fraction of the current width; <= 0 = default (0.002) */
    float   max_ratio;      /* cumulative expansion cap vs the stuck width; <= 0 = default (0.03) */
    int32_t max_rounds;     /* expansion rounds per relief; <= 0 = default (12) */
    int32_t strike_limit;   /* separator strike limit per expansion round; <= 0 = default (1) */
    int32_t round_budget_ms;/* wall-clock budget per expansion round in ms; <= 0 = unlimited (default).
                             * The strike limit alone still lets one strike burn the phase's whole
                             * iter_no_imprv_limit, so cap each round here for second-scale budgets. */
} CWidthReliefConfig;

/**
 * Closed-loop control of the exploration shrink step (ABI v5 tail block).
 *
 * `adaptive_shrink_gain` scales the step by density alone, which is open loop:
 * density rises monotonically as the strip narrows, so the multiplier is a fixed
 * schedule that never learns a step was too large. This block adds the feedback
 * the compression phase already has - back the step off when a width cannot be
 * separated, earn it back when a new feasible width is reached.
 *
 * Only the FIRST failure at a width counts, so repeated disruption retries at one
 * width cannot collapse the step to its floor.
 *
 * Adaptive-only; ignored by libraries built without the adaptive feature.
 */
typedef struct {
    float fail_decay; /* step-scale multiplier on the first failure at a width; must be in (0,1)
                       * to take effect, 1.0 or out of range = disabled (default) */
    float scale_min;  /* lower bound for the step scale; <= 0 or > 1 = default (0.05) */
} CShrinkFeedbackConfig;

/** Compression phase configuration. */
typedef struct {
    float            shrink_range_min;              /* legacy name; pair is order-independent, effective stop=min(pair) */
    float            shrink_range_max;              /* legacy name; pair is order-independent, effective start=max(pair) */
    float            time_limit_secs;
    int32_t          shrink_decay_strategy;         /* 0 = TimeBased, 1 = FailureBased */
    float            shrink_decay_ratio;            /* only used when FailureBased */
    CSeparatorConfig separator;
} CCompressionConfig;

/**
 * Polish (post-processing) phase configuration.
 *
 * Applied to the feasible solution produced by the compression phase. Enabled
 * LP/slide passes retry until time_limit_secs expires or a stop is requested:
 *   1) lp        - global LP compaction over rigid-unit translations and strip width
 *   2) slide     - recover small gaps after LP by pushing items left/down
 * Legacy hole-fill fields are retained for ABI compatibility and have no effect.
 *
 * Every pass verifies collisions before committing a move, so the result is
 * never worse than the input solution. Frozen items are never moved.
 */
typedef struct {
    int32_t enabled;                       /* 0 = off, otherwise on */
    float   time_limit_secs;
    int32_t n_rounds;                      /* Legacy ABI field; active rounds use the time budget. */

    int32_t slide_enabled;
    int32_t slide_n_passes;
    int32_t slide_down;                    /* 0 = left direction only */
    float   slide_safety_ratio;            /* clearance left on contact, relative to item min side */

    int32_t hole_fill_enabled;
    float   hole_fill_cell_size_ratio;     /* grid cell size, relative to smallest item side */
    int32_t hole_fill_max_candidates;
    float   hole_fill_candidate_band;      /* fraction of the strip's right side to consider (0.0-1.0) */
    int32_t hole_fill_n_samples;           /* samples evaluated per pocket */

    int32_t lp_enabled;
    int32_t lp_n_iterations;
    float   lp_trust_region_ratio;         /* base (finest) trust-region step, relative to strip width;
                                              grown x2 whenever progress stalls */
    float   lp_left_pull_weight;           /* weight of the "pull left" objective term */
} CPolishConfig;

/**
 * LBF starting-placement candidates from a raster no-fit map (capmods builds only).
 *
 * The layout and the item are rasterized onto the same grid so that collision-free grid
 * positions can be enumerated directly instead of relying on uniform random sampling.
 * Every candidate is still verified by the exact CDE, so grid resolution only affects
 * candidate quality, never correctness. Items placed in the last 10% of the LBF pass are
 * additionally slid into contact using an exact NFP refinement.
 *
 * On by default.
 */
typedef struct {
    int32_t enabled;           /* 0 = off, otherwise on (default) */
    float   cell_ratio;        /* cell = (smallest item's min side) * ratio; <=0 = default (0.12) */
    int32_t max_cells;         /* grid cell cap; <=0 = default (250000) */
    int32_t n_candidates;      /* raster candidates handed to the CDE; <=0 = default (64) */
    int32_t n_random_samples;  /* random samples kept alongside; 0 = raster only, <0 = default (1000) */
} CRasterNfpConfig;

/**
 * Experimental P0-P5 "structure first, search assisted" pipeline.
 *
 * This block is the ABI tail of CSwarmConfig: all legacy field offsets stay
 * unchanged and a zero-initialized trailing block keeps the legacy pipeline.
 * This is source/layout-offset compatibility, not compatibility with an
 * already-compiled caller that allocates the older, shorter struct; rebuild
 * such callers with this header. Validate swarm_get_abi_info() before
 * swarm_get_default_config(), then override only the fields you need.
 * Automatic motifs are reversible solver hints and are not public rigid
 * PlaceGroups.
 */
typedef struct {
    int32_t enabled;                    /* 0 = legacy (default), otherwise enable structure phase */
    int32_t motif_enabled;              /* P0 automatic soft motifs */
    int32_t max_active_motifs;          /* <=0 = default */
    int32_t construction_starts;        /* P1 independent starts; <=0 = default */
    float   profile_row_height_ratio;   /* P1 row height / smallest item side; <=0 = default */
    int32_t profile_top_k;              /* P1 stochastic candidate pool; <=0 = default */

    float   p2_time_limit_secs;         /* short preview compression budget; <0 = default */
    float   p2_shrink_range_min;        /* legacy name; <=0 = default, effective stop=min(pair) */
    float   p2_shrink_range_max;        /* legacy name; <=0 = default, effective start=max(pair) */
    float   p2_failure_decay;           /* 0 < value < 1; otherwise default */

    float   p3_slack_ratio;             /* warm-start slack; 0.05 = five percent */
    float   p3_time_limit_secs;         /* <0 = use normal exploration budget */

    int32_t p4_cycles;                  /* 0 = skip P4; <0 = default */
    float   p4_cycle_time_limit_secs;   /* per-cycle budget; <0 = default */
    float   p4_ruin_fraction;           /* fraction of strip targeted; invalid = default */
    int32_t p4_max_items;               /* <=0 = default */
    int32_t p4_stagnation_cycles;       /* <=0 = default */

    int32_t p5_snap_rotations;          /* non-zero = enforce input allowed angles */

    float   p1_time_limit_secs;         /* shared P0/P1 wall-clock budget; <0 = default */
    int32_t profile_max_rows;           /* profile raster row cap; <=0 = default */
    int32_t item_beam_width;            /* remaining item/copy choices per P1 step */
    int32_t candidate_pool_cap;         /* exact-authorized proposals per item */
    int32_t slide_refine_steps;         /* exact slide-to-contact step cap */
    int32_t p1_include_legacy_baseline; /* non-zero = paired legacy LBF fallback */
    int32_t pair_max_offset_candidates; /* P0 offsets per oriented pair */
    int32_t pair_max_orientation_work;  /* P0 global pair-orientation work cap */
    int32_t diagnostics_max_raster_cells;      /* Legacy ABI field; ignored (structure diagnostics removed) */
    int32_t diagnostics_max_pair_comparisons;  /* Legacy ABI field; ignored (structure diagnostics removed) */
    int32_t progress_callback_interval_ms;     /* All phase preview intervals; default 2000, <500 is clamped to 500 */
} CStructureFirstConfig;

/** Optional diagnostics attached to progress callback JSON. */
typedef struct {
    int32_t collect_loss;              /* non-zero = include `loss`; default on */
    int32_t collect_collision_status;  /* non-zero = include `collision_status`; default on */
} CCallbackOptionsConfig;

/**
 * N06 independent initial-placement engine settings (swarm_init_phase).
 * Pipeline: GRASP construction -> slide recompaction -> repeated
 * [width shrink -> overlap separation (rescue / stall exit / fine CD) ->
 * recompaction], with `kicks` re-expanding the shrink step until the time
 * budget runs out. Legacy (single-strip) inputs only.
 * Zero/negative fields fall back to the confirmed defaults noted per field.
 */
typedef struct {
    int32_t enabled;           /* 0 = disabled (default) */
    float   time_limit_secs;   /* <=0 = 60 */
    int32_t grasp;             /* <=0 = 1 construction start */
    int32_t kicks;             /* negative = 100000, 0 = no kicks */
    float   step0_frac;        /* <=0 = 0.005 (0.5% of width) */
    float   min_frac;          /* <=0 = 0.0005 */
    int32_t iter_cap;          /* <=0 = 3000 separation iterations */
    int32_t samples_base;      /* <=0 = 50 move samples */
    float   growth;            /* <=1 = 1.1 GLS weight growth */
    int32_t rescue_after;      /* negative = 300, 0 = off (exhaustive relocation period) */
    int32_t stall_cap;         /* negative = 1500, 0 = plain iteration-cap exit */
    float   cd_min;            /* <=0 = 0.03 mm coordinate-descent minimum step */
    int32_t compact_passes;    /* negative = 6, 0 = recompaction off */
    int32_t reseat_top;        /* negative = 4, 0 = reseat off */
    int32_t cand_eval_cap;     /* <=0 = 32 construction candidates */
    float   lbf_x_weight;      /* <=0 = 500 */
    float   lbf_y_weight;      /* negative = 0.1 */
} CInitialPlacementConfig;

/** ABI tail: escape. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t enabled;
    int32_t window_floor;
    int32_t window_item_factor;
    float efficiency_eps;
    float diversity_threshold;
    int32_t enable_conflict_burst;
    int32_t enable_swarm_diversification;
    int32_t max_push_items;
    int32_t max_infeasible_pool;
} CEscapeConfig;

/** ABI tail: nfp_lbf. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t enabled;
    int32_t n_orderings;
    int32_t cache_budget_ms;
    int32_t total_budget_ms;
    int32_t allow_180;
    int32_t enable_prefix_reuse;
    float concave_simplify_tolerance;
} CNfpLbfConfig;

/** ABI tail: fabric_tolerance. See swarm_get_default_config for defaults. */
typedef struct {
    float piece_overlap_mm;
} CFabricToleranceConfig;

/** ABI tail: nfp_integration. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t ordering_exploration;
    int32_t nfp_sampler;
    int32_t exact_separation_vector;
    int32_t nfp_hole_filling;
    int32_t nfp_reinsertion;
} CNfpIntegrationConfig;

/** ABI tail: nfp_lbf_tuning. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t max_candidate_probes;
    int32_t _reserved[3];
} CNfpLbfTuningConfig;

/** ABI tail: nfp_strategy. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t instance_rotations;
    int32_t continuous_rotation_step_deg;
    int32_t pairwise_clustering;
    float cluster_min_gain;
    int32_t contact_maximization;
    int32_t pocket_first;
    int32_t beam_search;
    int32_t beam_width;
    int32_t best_fit_selection;
    int32_t best_fit_lookahead;
    int32_t sliding_compaction;
    int32_t sliding_rounds;
} CNfpStrategyConfig;

/** ABI tail: lp_solver. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t backend;
    float quadratic_regularization;
    int32_t fallback;
    int32_t time_limit_ms;
} CLpSolverConfig;

/** ABI tail: nfp_strategy2. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t accretion;
    int32_t tail_reflow;
    int32_t reflow_rounds;
    int32_t _reserved;
} CNfpStrategy2Config;

/** ABI tail: refplace. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t enabled;
    int32_t group_fallback;
    int32_t _reserved0;
    float simplify_max_deviation_mm;
} CRefPlaceConfig;

/** ABI tail: lbf_order. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t mode;
    int32_t _reserved0;
    int32_t _reserved1;
    int32_t _reserved2;
} CLbfOrderConfig;

/** ABI tail: hole_first. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t enabled;
    float share;
    int32_t _reserved0;
    int32_t _reserved1;
} CHoleFirstConfig;

/** ABI tail: reclaim. See swarm_get_default_config for defaults. */
typedef struct {
    int32_t enabled;
    float compress_share;
    int32_t max_cycles;
    int32_t min_slice_ms;
    int32_t mode;
    float ruin_band_fraction;
    int32_t ruin_max_items;
    int32_t stagnation_cycles;
} CReclaimConfig;

/** ABI v16: Explore 분리 정체 복구. 세 스위치 모두 기본 꺼짐.
 *
 * Explore 의 분리 한 번 안에서, 전체 loss 가 그때까지의 최소값보다 0.5% 이상 낮아지지 못한
 * 반복이 stagnation_iterations 번 쌓이면 복구 패스를 한 번 돌린다(분리당 최대 max_passes 번).
 * 패스는 충돌이 큰 이동 단위(강체 그룹 = 한 단위) max_units 개를 대상으로, 켜진 방법을
 * 접촉 방향 -> 집단 LP -> 차단 연쇄 순으로 시도해 처음 채택된 후보에서 끝낸다.
 * 후보는 사본에서만 평가하고, 가중 loss 가 줄고 비가중 loss 가 늘지 않으며 스트립 폭이
 * 그대로일 때만 반영한다. 회전은 바꾸지 않는다. Init/Compress/Polish 에는 적용되지 않는다.
 *
 * 정수 예산은 아래 범위로 잘린다. max_move_mm 이 유한한 양수가 아니면 2mm 를 쓰고,
 * 100mm 를 넘으면 100mm 로 자른다.
 */
typedef struct {
    int32_t direction_enabled;      /* 접촉 방향: 충돌 단위 하나를 정점-변 접촉 후보 방향으로 옮겨 본다 */
    int32_t cluster_lp_enabled;     /* 집단 LP: 2*max_move_mm 안의 주변 단위들을 선형계획으로 동시에 옮긴다 */
    int32_t chain_enabled;          /* 차단 연쇄: 옮기면 새로 부딪히는 이웃까지 따라 옮긴다 */
    uint32_t stagnation_iterations; /* 복구를 켜는 정체 반복 수. 1..10000, 기본 24 */
    uint32_t max_passes;            /* 분리 한 번당 복구 패스 수 상한. 1..8, 기본 2 */
    uint32_t max_units;             /* 이동 단위 수 상한(대상 수/LP 집단 크기/연쇄 길이). 2..8, 기본 4 */
    uint32_t max_evaluations;       /* 패스 하나의 후보 평가 수 상한. 1..256, 기본 64 */
    uint32_t time_budget_ms;        /* 패스 하나의 벽시계 예산(ms). 1..200, 기본 20 */
    float max_move_mm;              /* 단위당 최대 이동 거리(mm). 0 초과 100 이하, 기본 2. 겹침 허용량이 아니다 */
} CSeparationRepairConfig;

/** ABI v17 tail: 구간(밴드) 통째 재배열. 전부 0 = 꺼짐/네이티브 기본값(v16 과 같은 동작). */
typedef struct {
    int32_t post_shrink;              /* 0 = 꺼짐. 그 외 = 축소에 성공할 때마다 우측 밴드를 통째로 다시 배열한다 */
    int32_t full_band;                /* 0 = 꺼짐. 그 외 = 밴드 안 이동 가능 조각을 전부 파괴한다 */
    float   recreate_x_mult;          /* 재삽입 LBF 평가기의 x 편향. <=0 = 기본 10.0 */
    float   recreate_y_mult;          /* 재삽입 LBF 평가기의 y 편향. <=0 = 기본 1.0. 둘 다 양수일 때만 반영 */
    float   min_clear_reinsert_ratio; /* 재배열을 유지할 최소 빈자리 착석률(0~1). <=0 = 검사 안 함 */
    int32_t pocket_band;              /* 0 = 꺼짐. 그 외 = 밴드 시작점 후보에 실제 빈틈 위치를 추가한다 */
    int32_t pocket_band_max;          /* pocket_band 로 추가할 포켓 수 상한. <=0 = 기본 24 */
    float   pocket_grid_cell_scale;   /* 빈틈 격자 셀 크기 배율. <=0 = 1.0 */
    float   max_loss_growth;          /* 재배열 뒤 loss 악화 허용 배율. <=0 = 검사 안 함 */
    int32_t post_strategy;            /* 0=Rightmost 1=LowDensity 2=Fragmented 3=GapTarget 4=DensitySeam 5=Random */
} CBandRearrangeConfig;

/** ABI v17 tail: 원본 외곽선이 원단 밖으로 넘어도 되는 최대 거리.
 *
 * 조각 겹침을 허용하면 충돌 도형과 컨테이너가 같은 양만큼 줄지만, 날카로운 꼭짓점의 원본
 * 외곽선은 그보다 더 밖으로 나간다. 값을 주면 각 단계의 결과를 공개하기 직전에 넘은 조각만
 * 안쪽으로 밀고, 밀 수 없는 우측 끝 초과는 스트립 길이를 그만큼 늘려 없앤다.
 */
typedef struct {
    float margin_mm;                  /* <=0 또는 비유한 = 검사 안 함(기본) */
} CFabricEdgeConfig;

/** ABI v17 tail: GLS 쌍 가중치의 학습률과 상한. 전부 0 = 네이티브 상수. */
typedef struct {
    float min_inc;                    /* 충돌 중인 쌍의 최소 증가율. <=1 = 기본 1.2 */
    float max_inc;                    /* 충돌 중인 쌍의 최대 증가율. <min_inc = 기본 2.0 */
    float decay;                      /* 충돌하지 않는 쌍의 감쇠율. <=0 또는 >1 = 기본 0.95 */
    float max_weight;                 /* 가중치 상한. <=1 = 기본 1e6 */
} CGlsWeightConfig;

/** ABI v17 tail: 초기 구성기의 배치 규칙. 기본값은 swarm_get_default_config 가 채운다
 *  (가중치 셋은 -1 = 네이티브 기본값). */
typedef struct {
    int32_t double_sided_first_n;     /* 0 = 꺼짐. 그 외 = 앞쪽 N 조각에 양쪽 모서리 앵커링 */
    int32_t settle_every;             /* 구성 도중 몇 조각마다 다질지. <=0 = 하지 않음 */
    int32_t settle_budget_ms;         /* 다지기 한 번의 예산(ms). <=0 = 기본 200 */
    int32_t post_init_compaction_ms;  /* 구성 직후 한 번 다질 예산(ms). <=0 = 하지 않음 */
    float   band_width_mm;            /* 구간 완성 폭(mm). <=0 = 끔 */
    int32_t nfp_score_first;          /* 0 = 끔. 그 외 = NFP 구성형 배치에서 가중합 점수를 1순위로 */
    float   nfp_w2_waste;             /* 낭비 면적 가중치. 음수/비유한 = 기본 0.35 */
    float   nfp_w3_contact;           /* 접촉 길이 가중치. 음수/비유한 = 기본 0.25 */
    float   nfp_w4_y;                 /* 세로 위치 가중치. 음수/비유한 = 기본 0.10 */
    float   nfp_length_tie_mm;        /* 길이 동률 허용치(mm). <=0 = 끔 */
    float   band_fill_ratio;          /* 구간을 채울 작은 조각의 면적 상한(가장 큰 조각 대비). <=0 = 훑지 않음 */
} CConstructorConfig;

/** ABI v18 tail: 탐색 단계 정체 대응. 전부 0 = v17 과 같은 동작. */
typedef struct {
    int32_t sep_restart_window;       /* Explore 분리기의 정체 재출발 창(라운드). <=0 = 꺼짐 */
    int32_t sep_iter_floor;           /* Explore 분리기 iter_no_imprv_limit 하한 덮어쓰기. <=0 = 종전 */
    int32_t cmpr_restart_window;      /* Compress 분리기의 정체 재출발 창. <=0 = 꺼짐 */
    int32_t rnr_original_candidate;   /* 예약(RESERVED) — 이 빌드의 코어는 아직 소비하지 않는다(미구현).
                                         의도: 0 = 꺼짐. 그 외 = R&R 재삽입에서 원위치를 후보로 포함 */
    int32_t escape_window_absolute;   /* 0 = 종전. 그 외 = 탈출 파이프라인의 정체 창을 절대값으로 */
    int32_t sep_time_cap_ms;          /* Explore 분리 1회 벽시계 상한(ms). 0 = 제한 없음.
                                         -1 = 적응형(가장 오래 걸린 성공 분리의 1.25배). 그 밖의 음수 = 제한 없음 */
    int32_t portfolio_k;              /* 예약(RESERVED) — 이 빌드의 코어는 아직 소비하지 않는다(미구현).
                                         의도: 분리 실패 뒤 탐침할 교란 후보 수. <=1 = 종전 */
    int32_t probe_budget_ms;          /* 예약(RESERVED) — 이 빌드의 코어는 아직 소비하지 않는다(미구현).
                                         의도: 교란 후보 하나의 탐침 예산(ms). <=0 = 기본 2000 */
} CSearchTuningConfig;

/** Excess original-outline overlap handling (ABI v19 tail block).
 *
 *  With piece overlap the search uses pieces shrunk by half the allowance and the
 *  result is re-checked on the ORIGINAL outlines. Sharp corners shrink more than
 *  edges, so original outlines can overlap deeper than the allowance even when the
 *  shrunk shapes do not ("[ORIGINAL:VALIDATE] original overlap exceeds configured
 *  piece_overlap_mm"). */
#define SWARM_ORIGINAL_OVERLAP_INFEASIBLE 0  /* reject such layouts (default, v18 behavior) */
#define SWARM_ORIGINAL_OVERLAP_FEASIBLE   1  /* accept them: original-outline check off */
typedef struct {
    int32_t excess_policy;   /* 0 = 실현 불가능(기본, 버림). 1 = 실현 가능(채택). 그 밖의 값 = 0 */
} COriginalOverlapConfig;

/** V2 place-group representation (existing native ABI v20 tail). */
#define SWARM_PLACE_GROUP_MULTIPOLYGON 0
#define SWARM_PLACE_GROUP_RIGID 1
typedef struct {
    int32_t mode; /* 0 = multipart item (default), 1 = rigid member movement */
} CPlaceGroupConfig;

/** Full swarm optimizer configuration. */
typedef struct {
    uint64_t           rng_seed;                    /* 0 = random */
    float              poly_simpl_tolerance;        /* negative = None */
    float              min_item_separation;         /* negative = None */
    float              narrow_concavity_cutoff_dist;/* negative = None */
    float              narrow_concavity_cutoff_area;/* negative = None */
    float              lbf_x_multiplier;            /* LBF X-axis bias (default 10.0) */
    float              lbf_y_multiplier;            /* LBF Y-axis bias (default 1.0) */
    CExplorationConfig exploration;
    CCompressionConfig compression;
    CPolishConfig      polish;
    CRasterNfpConfig   lbf_raster;
    int32_t            allow_180_rotation;          /* 0 = off (default); otherwise add 180-deg flips to each item's allowed rotations. Non-nap fabrics only. */
    float              tilt_range_deg;              /* small-angle tilt (+/- deg) allowed around discrete rotations; <=0 = off (default). Keep 0 for nap fabrics. */
    CStructureFirstConfig structure_first;           /* ABI-tail experimental P0-P5 controls; disabled by default */
    CCallbackOptionsConfig callback_options;         /* ABI-tail callback-only diagnostics; solver validation is unaffected */
    CInitialPlacementConfig initial_placement;       /* ABI-tail N06 initial-placement engine; disabled by default */
    CWidthReliefConfig   width_relief;              /* ABI-tail exploration width relief; disabled by default */
    CShrinkFeedbackConfig shrink_feedback;          /* ABI-tail exploration shrink-step feedback; disabled by default */
    CEscapeConfig escape;
    CNfpLbfConfig nfp_lbf;
    CFabricToleranceConfig fabric_tolerance;
    CNfpIntegrationConfig nfp_integration;
    CNfpLbfTuningConfig nfp_lbf_tuning;
    CNfpStrategyConfig nfp_strategy;
    CLpSolverConfig lp_solver;
    CNfpStrategy2Config nfp_strategy2;
    CRefPlaceConfig refplace;
    CLbfOrderConfig lbf_order;
    CHoleFirstConfig hole_first;
    CReclaimConfig reclaim;
    CSeparationRepairConfig repair;
    CBandRearrangeConfig band_rearrange;             /* ABI v17 tail */
    CFabricEdgeConfig fabric_edge;                   /* ABI v17 tail */
    CGlsWeightConfig gls;                            /* ABI v17 tail */
    CConstructorConfig constructor;                  /* ABI v17 tail */
    CSearchTuningConfig search;                      /* ABI v18 tail */
    COriginalOverlapConfig original_overlap;         /* ABI v19 field (offset 948) */
    CPlaceGroupConfig place_group;                    /* ABI v20 tail (size 960) */
} CSwarmConfig;

/**
 * Fixed-width ABI handshake. Call swarm_get_abi_info() and validate this
 * record before passing a caller-allocated CSwarmConfig to the library.
 */
typedef struct {
    uint32_t abi_version;
    uint32_t swarm_config_size;
    uint32_t structure_config_size;
    uint32_t feature_flags;
} CSwarmAbiInfo;

/**
 * Optional starting placement for an item.
 *
 * Sending a hint always seeds the item at (tr_x, tr_y, rot_angle_in_rad) — that is the
 * point of sending one. is_freeze is a separate, additional constraint on top of it:
 *   is_freeze = 0 : start here, but exploration/compression may move it (warm start)
 *   is_freeze = 1 : start here and never move
 * Frozen/frozen overlap and frozen/strip-exterior conflicts are permitted in
 * Init, Explore, Compress, Polish, validation, callbacks and exports. Movable
 * placements must still avoid frozen placements and remain inside the strip.
 * Items without a hint are placed by the LBF constructor as before, so hinting a subset
 * seeds those and lets LBF fill in the rest.
 * A hint whose item_id is out of range is ignored.
 */
typedef struct {
    int32_t item_id;
    int32_t is_freeze;
    float   tr_x;             /* x of the item's bbox bottom-left corner (mm) */
    float   tr_y;             /* y of the item's bbox bottom-left corner (mm) */
    float   rot_angle_in_rad;
    float   stripe_interval_y; /* snap the placed bbox bottom-left Y to this global grid; <=0 = off */
    float   stripe_interval_x; /* snap the placed bbox bottom-left X to this global grid; <=0 = off */
} CPlacedHint;

/** Axis-aligned bounds used by swarm_find_bound_collisions(). */
typedef struct {
    int32_t item_id;
    float   min_x;
    float   min_y;
    float   max_x;
    float   max_y;
} CItemBounds;

/** Per-place collision state returned by swarm_get_collision_status(). */
typedef struct {
    size_t  item_id;
    uint8_t collision;
} CCollisionStatus;

/* ════════════════════════════════════════════════════════════════
 *  Callback type
 * ════════════════════════════════════════════════════════════════ */

/**
 * Progress callback, invoked during optimization whenever a solution is reported.
 *
 * @param report_type   0=ExplFeas, 1=ExplInfeas, 2=ExplImproving, 3=CmprFeas, 4=Final.
 *                      Structure checkpoints map P1/P3->0, P2->3, P4->2, P5->4.
 * @param strip_width   current strip width
 * @param density       current density (0.0 - 1.0)
 * @param solution_json combined JSON object (UTF-8, null-terminated) containing
 *                      `solution` and the callback_options-enabled diagnostics
 *                      (`loss`, `collision_status`) for this report,
 *                      or NULL when none is attached (e.g. feasibility-probe reports,
 *                      throttled reports, or when emit_intermediate_json is 0).
 *                      Valid ONLY during the callback — the library frees it right
 *                      after the callback returns, so copy it if you need to keep it.
 * @param user_data     opaque pointer passed through from swarm_set_callback()
 *
 * Reentrancy contract: only swarm_request_stop() and
 * swarm_request_stop_and_wait() are safe to call with this handle from inside
 * the callback. Do not call phase functions, getters, setters, freeze, or free
 * from the callback; perform those operations after it returns.
 */
typedef void (*SolutionCallback)(
    int32_t     report_type,
    float       strip_width,
    float       density,
    const char *solution_json,
    void       *user_data
);

/* ════════════════════════════════════════════════════════════════
 *  Functions
 * ════════════════════════════════════════════════════════════════ */

/**
 * Fill the ABI/feature handshake. Returns 0 on success or -1 when info is NULL.
 * Older libraries without this export are ABI-incompatible with this header.
 */
int32_t swarm_get_abi_info(CSwarmAbiInfo *info);

/**
 * Fill a CSwarmConfig struct with the library's default configuration.
 * @param config Pointer to a caller-allocated CSwarmConfig to populate.
 */
void swarm_get_default_config(CSwarmConfig *config);

/**
 * Free a string previously returned by the library (e.g. output_json, error_message).
 * @param ptr Pointer to the string to free (may be NULL).
 */
void swarm_free_string(char *ptr);

/** Error code for the most recent init, phase, or solution-export call on this thread (0 = none). */
int32_t swarm_get_last_error_code(void);

/**
 * UTF-8 detail for the most recent init, phase, or solution-export error on this thread.
 * Free the returned string with swarm_free_string().
 */
char *swarm_get_last_error_message(void);

/**
 * Initialize a phase handle from an input file path.
 * Returns NULL on invalid input or initialization failure.
 */
void *swarm_init(
    const char *input_path,
    const CSwarmConfig *config,
    const CPlacedHint *hints,
    int32_t hint_count
);

/**
 * Initialize a phase handle from an input/output JSON string or a schema_version=2
 * all_items/size_regions envelope. A V2 envelope owns all region lanes behind this
 * single returned handle. On NULL, read swarm_get_last_error_* on this thread.
 */
void *swarm_init_from_json(
    const char *input_json,
    const CSwarmConfig *config,
    const CPlacedHint *hints,
    int32_t hint_count
);

/* Additive direct-input ABI, independently versioned from CSwarmConfig.
 * V2 semantics: IDs 0..item_count-1, demand=1 per placement. No packing pragmas.
 * Pointer/size_t pairs borrow aligned readable buffers for the call only.
 * Empty buffers may be NULL. Strings are UTF-8 byte slices, without a required NUL.
 * All data is copied into owned Swarm input before return. Geometry validation,
 * simplification and initial placement happen at solver initialization, not prepare.
 */
#define SWARM_INPUT_ABI_VERSION UINT32_C(1)
typedef struct { float x, y; } CInputPoint;
typedef struct {
    int32_t ver;
    int64_t item_id;
    int32_t is_freeze;
    float tr_x, tr_y, rot_angle_in_rad, stripe_interval_y, stripe_interval_x;
} CInputHint;
typedef struct {
    uint64_t id, demand;
    uint32_t shape_kind;    /* 0 rectangle, 1 simple polygon; other kinds rejected */
    uint32_t rotation_kind; /* 0 discrete, 1 stepped, 2 continuous */
    float x_min, y_min, width, height; /* rectangle only */
    const CInputPoint *vertices;
    size_t vertex_count;
    const float *angles;    /* discrete rotation, degrees; not legacy empty-list semantics */
    size_t angle_count;
    float rotation_step;   /* stepped rotation, degrees */
    const float *reflection_axes;
    size_t reflection_count;
    int32_t min_quality;   /* -1 = absent */
    uint32_t has_hint;     /* 0 absent, 1 present */
    CInputHint hint;
} CInputItem;
typedef struct {
    const uint8_t *id;
    size_t id_len;
    const uint8_t *kind; /* size_group, size_bundle, remainder */
    size_t kind_len;
    const uint64_t *item_ids;
    size_t item_count;
    float gap_after_mm;
} CInputRegion;
typedef struct {
    int64_t id;
    const uint64_t *item_ids;
    size_t item_count;
} CInputPlaceGroup;
typedef struct {
    uint32_t abi_version;  /* SWARM_INPUT_ABI_VERSION */
    uint32_t struct_size;  /* sizeof(CSwarmInput) */
    const uint8_t *name;
    size_t name_len;
    float strip_height, min_item_separation;
    const CInputItem *items;
    size_t item_count;
    uint32_t mode;         /* 0 all_items, 1 size_regions */
    uint32_t group_workers;/* >= 1 */
    uint64_t quantum_ms;   /* 1..60000 */
    float locked_prefix_gap_mm;
    const CInputRegion *regions;
    size_t region_count;
    const CInputPlaceGroup *place_groups;
    size_t place_group_count;
} CSwarmInput;
typedef struct SwarmInput SwarmInput;

/* Prepare only: returns owned validated V2 input, or NULL + thread-local last error.
 * swarm_input_from_json accepts V2 envelopes; swarm_init_from_json still accepts legacy too.
 * Neither preparation function creates an optimizer or imports geometry. */
SwarmInput *swarm_input_from_json(const char *json);
SwarmInput *swarm_input_from_struct(const CSwarmInput *input);
/* NULL is allowed. Never pass a solver handle to this function. */
void swarm_input_free(SwarmInput *input);
/* Consumes input even on failure; do not free or reuse it after this call.
 * Returns an ordinary solver handle, released with swarm_free. config may be NULL. */
void *swarm_init_from_input(SwarmInput *input, const CSwarmConfig *config);
/* Convenience prepare + init. The caller's descriptors remain owned by the caller. */
void *swarm_init_from_struct(const CSwarmInput *input, const CSwarmConfig *config);

/** Register or clear the callback used by the phase functions. */
void swarm_set_callback(
    void *handle,
    SolutionCallback callback,
    void *user_data,
    int32_t emit_json
);

/** 현재 모든 Place의 충돌 상태를 JSON 형식으로 반환. 반환된 문자열은 호출자가 해제해야 함.*/
char *swarm_get_collision_status_json(const void *handle);

/** Return per-place collision states. Free the returned array with swarm_free_collision_status(). */
CCollisionStatus *swarm_get_collision_status(const void *handle, size_t *count);

/** Free an array returned by swarm_get_collision_status(). */
void swarm_free_collision_status(CCollisionStatus *statuses, size_t count);

/**
 * Run the N06 initial-placement phase. Optional; only valid directly after
 * swarm_init(_from_json)() and only when
 * CSwarmConfig.initial_placement.enabled is non-zero. Replaces the LBF
 * starting solution with the N06 engine result and keeps the handle in the
 * Init phase so swarm_alg_phase()/swarm_cp_phase() can follow.
 * Legacy (single-strip) inputs only; grouped/V2 inputs fail with error 3004.
 * Cooperatively cancellable via swarm_request_stop().
 * Returns 0 on success, non-zero on failure.
 */
int32_t swarm_init_phase(void *handle);

/**
 * Run the exploration phase.
 *
 * Explore, Compress and Polish may be called in ANY order and ANY number of
 * times after swarm_init() (e.g. Explore -> Explore -> Compress -> Polish ->
 * Explore -> Compress). Every call starts from the certified current solution
 * left by the previous call and runs for its own time limit; the result never
 * gets worse. None of them is valid after swarm_gp_phase().
 * Returns 0 on success, non-zero on failure.
 */
int32_t swarm_alg_phase(void *handle);

/**
 * Run the compression phase. Free order/repetition — see swarm_alg_phase().
 * Called right after swarm_init() it compresses the LBF starting solution.
 * Returns 0 on success, non-zero on failure.
 */
int32_t swarm_cp_phase(void *handle);

/**
 * Run the polish phase (LP compaction, deterministic slide, then bay/hole fill).
 * Free order/repetition — see swarm_alg_phase().
 * Returns 0 on success, non-zero on failure.
 */
int32_t swarm_refine_phase(void *handle);

/**
 * Run the complete P0-P5 structure-first alternative pipeline.
 * Must be called directly after swarm_init() and requires
 * config.structure_first.enabled != 0. Existing callbacks receive P1-P5
 * intermediate solutions. Completed and cancelled calls retain only the last
 * exact-certified collision-free public solution. A cancelled call returns 0
 * with `cancelled=true` in the audit report; it does not emit fake later-stage
 * milestones. Returns non-zero on validation or execution failure.
 */
int32_t swarm_gp_phase(void *handle);

/**
 * Return the P0-P5 checkpoint/diagnostic audit JSON, including a cancelled
 * baseline/partial run, or NULL before Structure has produced an audit.
 * Free the returned string with swarm_free_string().
 */
char *swarm_get_structure_report_json(const void *handle);

/** Return an exact-certified final solution JSON. NULL means unavailable or validation failure.
 *  Free a non-NULL result with swarm_free_string(). */
char *swarm_get_solution_json(void *handle);

/** Return a JSON array of item IDs involved in bounds/polygon collisions. */
char *swarm_find_bound_collisions(
    const void *handle,
    const CItemBounds *bounds,
    int32_t count
);

/** Set the item IDs that must remain fixed during subsequent phases. */
void swarm_freeze_items(void *handle, const int32_t *ids, int32_t count);

/** Defaults for future handles; do not affect an active solver. */
int swarm_set_shrink_step(float step);
void swarm_clear_shrink_step(void);
float swarm_get_shrink_step(void);

/** Live, handle-local shrink control; safe during a running phase.
 * Setter returns 1 on success, 0 for invalid input/handle. Getter returns -1
 * when unset. As with stop, the caller must keep the handle alive. */
int swarm_set_handle_shrink_step(const void *handle, float step);
void swarm_clear_handle_shrink_step(const void *handle);
float swarm_get_handle_shrink_step(const void *handle);

/** Request cancellation. Cancellation is permanent for the lifetime of this handle. */
void swarm_request_stop(void *handle);

/** Cancel an in-flight swarm_init_from_json() from any thread. Init has no
 * handle yet, so swarm_request_stop() cannot reach it. The flag is process-wide
 * and cleared when the next swarm_init_from_json() begins; a cancelled Init
 * returns NULL with error code 2002 (user cancellation, not a failure). */
void swarm_cancel_init(void);

/** Return non-zero if stop has been requested on the handle. */
int swarm_is_stop_requested(const void *handle);

/**
 * Signal stop and block until the phase has actually exited.
 * Returns immediately when no phase is running. Cancellation is permanent for this handle.
 * From a phase callback it records cancellation and returns to avoid waiting on the callback thread.
 */
void swarm_request_stop_and_wait(void *handle);

/**
 * Stop any running phase, wait for active calls, and free the handle.
 * Do not begin new calls once this function has started, and call it exactly once.
 * Do not call this blocking function from a solution callback running on the phase thread.
 * If it is called there anyway, it requests stop and returns without freeing (the handle
 * leaks) instead of deadlocking on its own phase.
 */
void swarm_free(void *handle);

/**
 * 네이티브 로그 수신 함수.
 *   level     : 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Trace
 *   message   : NUL 종료 UTF-8. 콜백이 반환하는 즉시 무효다 — 보관하려면 복사할 것.
 *   user_data : swarm_set_log_callback 에 넘긴 값 그대로.
 * 아무 스레드에서나(탐색 워커 포함), 동시에도 불린다. 핸들러는 스레드 안전해야 한다.
 */
typedef void (*SwarmLogCallback)(int32_t level, const char *message, uintptr_t user_data);

/**
 * 로그 콜백을 등록한다. callback 이 NULL 이면 해제한다.
 * max_level: 0=Off, 1=Error, 2=Warn, 3=Info, 4=Debug, 5=Trace (릴리스 빌드는 3까지만 나온다).
 * 반환값 1 = 성공, 0 = 실패(다른 로거가 이미 프로세스에 설치돼 있음). 여러 번 불러도 된다.
 */
int32_t swarm_set_log_callback(SwarmLogCallback callback, uintptr_t user_data, int32_t max_level);

/** 콜백으로 넘기지 못하고 버린 로그 줄 수. 정상 동작에서는 0 이다. */
uint64_t swarm_log_dropped_count(void);

/**
 * Return the library version as a static C string.
 * The returned pointer must NOT be freed.
 * @return Static version string (e.g., "0.1.0").
 */
const char *swarm_version(void);

#ifdef __cplusplus
}
#endif

#endif /* SWARM_H */
