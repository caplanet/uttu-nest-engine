using ReactiveUI;
using ReactiveUI.SourceGenerators;
using Uttu.NestingV2;
using System;

namespace CAPlanetSwarmSample.WPF.Views;

public partial class NestingConfigViewModel : ReactiveObject
{
    public NestingConfigViewModel()
    {
        InitializePhaseEditor();
    }

    #region General

    /// <summary>
    /// 단일 실행의 시드. 화면에서는 상단의 시드 목록이 실행마다 이 값을 덮어쓴다
    /// (<c>WorkspaceViewModel.SeedValues</c>). 0 은 무작위.
    /// </summary>
    [Reactive] public partial ulong RngSeed { get; set; } = 0;//42;//12345;

    [Reactive] public partial float PolySimplTolerance { get; set; } = 0.0001f;//0.001f;
    [Reactive] public partial bool MmSimplificationEnabled { get; set; }
    [Reactive] public partial float SimplifyMaxDeviationMm { get; set; } = 0.5f;
    
    [Reactive] public partial bool MinItemSeparationEnabled { get; set; } = true;

    [Reactive] public partial float MinItemSeparation { get; set; } = 0f;//0.0f;
    
    [Reactive] public partial float NarrowConcavityCutoffDist { get; set; } = 0.01f;

    [Reactive] public partial float NarrowConcavityCutoffArea { get; set; } = 0.01f;

    /// <summary>LBF 초기 배치 X축 편향 배수. 클수록 좌측으로 강하게 몰린다.</summary>
    [Reactive] public partial float LbfXMultiplier { get; set; } = 5.0f;

    /// <summary>LBF 초기 배치 Y축 편향 배수. 클수록 하단으로 강하게 몰린다.</summary>
    [Reactive] public partial float LbfYMultiplier { get; set; } = 1.0f;

    #endregion

    #region LBF Raster NFP (capmods 전용)

    /// <summary>래스터 NFP 후보 생성 사용 여부. 초기해가 결과에 직결될 때(힌트 warm start, 짧은 예산)만 켠다.</summary>
    [Reactive] public partial bool RasterEnabled { get; set; } = true;

    [Reactive] public partial float RasterCellRatio { get; set; } = 0.12f;

    [Reactive] public partial int RasterMaxCells { get; set; } = 250_000;

    [Reactive] public partial int RasterNumCandidates { get; set; } = 64;

    [Reactive] public partial int RasterNumRandomSamples { get; set; } = 1000;

    #endregion

    #region 회전 자유도

    /// <summary>허용 회전에 180° 반전 추가. 결방향 없는(non-nap) 원단 전용. (WSjk 60초 +0.8%p)</summary>
    [Reactive] public partial bool Allow180Rotation { get; set; } = false;

    /// <summary>discrete 회전 아이템의 소각도 틸트 허용 (도, ±값). 0 = 비활성. 결방향 원단은 0. (WSjk 60초 ±2°에서 +2.2%p)</summary>
    [Reactive] public partial float TiltRangeDeg { get; set; } = 0f;

    #endregion

    #region Structure First (Experimental)

    /// <summary>실험적 P0-P5 structure-first 대체 파이프라인. 기본값은 비활성.</summary>
    [Reactive] public partial bool StructureFirstEnabled { get; set; } = false;

    /// <summary>structure-first P0의 soft motif 후보 생성을 사용한다.</summary>
    [Reactive] public partial bool StructureMotifEnabled { get; set; } = true;

    /// <summary>
    /// V2 size-region lane을 동시에 실행할 outer scheduler worker 수.
    /// 각 lane 내부 Explore/Compress worker와는 별도이며 1 이상이어야 한다.
    /// 기본값은 현재 프로세스에서 보이는 논리 프로세서 수이며, 실제 권장값은 movable lane 수와
    /// 논리 프로세서 수 중 작은 값이다. 입력 JSON의 quantum_ms는 별도 실행 계약으로 유지한다.
    /// </summary>
    [Reactive] public partial int GroupWorkers { get; set; } = Math.Max(1, Environment.ProcessorCount);

    #endregion

    #region Callback Options

    /// <summary>
    /// 중간 결과 callback 통지 간격(ms). Explore/Compress/Polish/Structure 모두에 적용된다.
    /// 기본값은 2000ms이며 500 미만은 500ms로 보정된다. 단계 종료 결과는 간격과 무관하게 즉시 전달된다.
    /// 프리셋이 속성 이름으로 저장되므로 이름은 유지한다.
    /// </summary>
    [Reactive] public partial int StructureProgressCallbackIntervalMs { get; set; } = 2000;

    /// <summary>콜백 JSON의 Loss 진단을 수집합니다. WPF에서는 사용하지 않아 기본 OFF입니다.</summary>
    [Reactive] public partial bool CallbackCollectLoss { get; set; } = false;

    /// <summary>콜백 JSON의 아이템별 충돌 상태를 수집해 화면 강조에 사용합니다.</summary>
    [Reactive] public partial bool CallbackCollectCollisionStatus { get; set; } = true;

    #endregion

    #region Exploration

    [Reactive] public partial float ExplShrinkStep { get; set; } = 0.001f;//0.002f;

    [Reactive] public partial float ExplTimeLimitSecs { get; set; } = 480f;//160f;
    
    /// <summary>
    /// 연속 실패 상한으로 탐색을 조기 종료할지. 기본 꺼짐(네이티브 기본값과 같다).
    /// 켜면 탐색이 벽에 닿은 뒤 <see cref="ExplMaxConseqFailedAttempts"/> 번 연속 실패한 시점에
    /// 남은 탐색 시간을 버리고 다음 단계로 넘어간다 — 긴 예산을 준 실행이 중간에 끝나 보이는 원인이다.
    /// </summary>
    [Reactive] public partial bool ExplMaxConseqEnabled { get; set; } = false;

    [Reactive] public partial int ExplMaxConseqFailedAttempts { get; set; } = 15;
    
    [Reactive] public partial float ExplSolutionPoolStdDev { get; set; } = 0.3f;
    
    [Reactive] public partial float ExplLargeItemCutoffPercentile { get; set; } = 0.7f;

    #endregion

    #region Exploration Adaptive (adaptive feature 전용)

    /// <summary>밀도 기반 적응형 shrink 강도. 0 = 고정 스텝(원본과 동일).</summary>
    [Reactive] public partial float AdaptiveShrinkGain { get; set; } = 4.0f;

    /// <summary>적응형 shrink의 ShrinkStep 대비 최대 배율.
    /// 기본 gain 4.0에서는 density &lt;= 0 에서만 걸리므로 실질적으로 발동하지 않는다.</summary>
    [Reactive] public partial float AdaptiveShrinkMaxMult { get; set; } = 5.0f;

    /// <summary>축소 스텝 실패 피드백: 한 폭에서 첫 실패 시 스텝 배율에 곱할 감쇠 계수.
    /// (0,1) 범위여야 동작. 1.0 = 비활성.</summary>
    [Reactive] public partial float ShrinkFailDecay { get; set; } = 1.0f;

    /// <summary>축소 스텝 실패 피드백: 스텝 배율의 하한.</summary>
    [Reactive] public partial float ShrinkScaleMin { get; set; } = 0.05f;

    /// <summary>폭 진동: 같은 폭 연속 실패 임계값. 0 = 비활성.</summary>
    [Reactive] public partial int OscFailThreshold { get; set; } = 4;

    /// <summary>폭 진동: 되돌릴 폭 (ShrinkStep 배수).
    /// 실제 확장량은 min(ShrinkStep × 이 값, headroom/2)로 잘리고 headroom은 언제나
    /// 직전 축소 한 스텝이라, AdaptiveShrinkMaxMult의 절반을 넘는 값은 무의미하다.</summary>
    [Reactive] public partial float OscExpandSteps { get; set; } = 3.0f;

    /// <summary>폭 완화 복구: 한 폭에서 이만큼 연속 실패하면 최선 충돌 배치를 유지한 채
    /// 폭을 조금씩 넓혀 충돌을 푼다. 0보다 크면 OscFailThreshold를 대체한다. 0 = 비활성.</summary>
    [Reactive] public partial int ReliefFailThreshold { get; set; } = 0;

    /// <summary>폭 완화 복구: 라운드 1회 확장 비율 (현재 폭 대비).</summary>
    [Reactive] public partial float ReliefStep { get; set; } = 0.002f;

    /// <summary>폭 완화 복구: 막힌 폭 대비 누적 확장 상한.</summary>
    [Reactive] public partial float ReliefMaxRatio { get; set; } = 0.03f;

    /// <summary>폭 완화 복구: 복구 1회의 확장 라운드 상한.</summary>
    [Reactive] public partial int ReliefMaxRounds { get; set; } = 12;

    /// <summary>폭 완화 복구: 확장 라운드 1회의 separation strike 한도.</summary>
    [Reactive] public partial int ReliefStrikeLimit { get; set; } = 1;

    /// <summary>폭 완화 복구: 확장 라운드 1회의 벽시계 예산(ms). 0 = 무제한.</summary>
    [Reactive] public partial int ReliefRoundBudgetMs { get; set; } = 0;

    #endregion

    #region Exploration Separator

    [Reactive] public partial int ExplSepIterNoImprvLimit { get; set; } = 300;
    
    [Reactive] public partial int ExplSepStrikeLimit { get; set; } = 5;
    
    [Reactive] public partial int ExplSepNumWorkers { get; set; } = 4;
    
    [Reactive] public partial int ExplSepNumContainerSamples { get; set; } = 100;
    
    [Reactive] public partial int ExplSepNumFocussedSamples { get; set; } = 50;
    
    [Reactive] public partial int ExplSepNumCoordDescents { get; set; } = 5;

    /// <summary>컨테이너 전역 샘플의 좌측 편향 최대 강도. 0 = 균등 분포. capmods 전용.</summary>
    [Reactive] public partial float ExplSepLeftBiasGain { get; set; } = 2.0f;

    #endregion

    #region Compression

    [Reactive] public partial float CmprShrinkRangeMin { get; set; } = 0.001f;
    
    [Reactive] public partial float CmprShrinkRangeMax { get; set; } = 0.00001f;

    [Reactive] public partial float CmprTimeLimitSecs { get; set; } = 120f;//40f;
    
    [Reactive] public partial ShrinkDecayStrategy CmprShrinkDecay { get; set; } = ShrinkDecayStrategy.TimeBased;
    
    [Reactive] public partial float CmprShrinkDecayRatio { get; set; } = 0.9f;

    #endregion

    #region Compression Separator

    [Reactive] public partial int CmprSepIterNoImprvLimit { get; set; } = 300;//150;

    [Reactive] public partial int CmprSepStrikeLimit { get; set; } = 14;//7;
    
    [Reactive] public partial int CmprSepNumWorkers { get; set; } = 4;
    
    [Reactive] public partial int CmprSepNumContainerSamples { get; set; } = 100;
    
    [Reactive] public partial int CmprSepNumFocussedSamples { get; set; } = 50;
    
    [Reactive] public partial int CmprSepNumCoordDescents { get; set; } = 5;

    /// <summary>컨테이너 전역 샘플의 좌측 편향 최대 강도. 0 = 균등 분포. capmods 전용.</summary>
    [Reactive] public partial float CmprSepLeftBiasGain { get; set; } = 2.0f;

    #endregion

    #region Polish (후처리)

    /// <summary>후처리 단계 전체 on/off.</summary>
    [Reactive] public partial bool PolishEnabled { get; set; } = true;

    /// <summary>후처리 단계 시간 제한(초).</summary>
    [Reactive] public partial float PolishTimeLimitSecs { get; set; } = 20f;

    /// <summary>기존 저장 설정 호환용. 활성 LP/slide는 시간 예산 또는 정지 요청까지 반복한다.</summary>
    [Reactive] public partial int PolishNumRounds { get; set; } = 3;

    /// <summary>좌측/하단 슬라이딩 패스 사용 여부.</summary>
    [Reactive] public partial bool SlideEnabled { get; set; } = true;

    /// <summary>한 라운드 안에서 슬라이딩을 반복할 횟수.</summary>
    [Reactive] public partial int SlideNumPasses { get; set; } = 3;

    /// <summary>좌측 슬라이딩 후 하단 슬라이딩도 수행할지.</summary>
    [Reactive] public partial bool SlideDown { get; set; } = true;

    /// <summary>접촉 시 남겨둘 안전 간격 (아이템 최소 변 대비 비율).</summary>
    [Reactive] public partial float SlideSafetyRatio { get; set; } = 0.0f;

    /// <summary>LP compaction 패스 사용 여부.</summary>
    [Reactive] public partial bool LpEnabled { get; set; } = true;

    /// <summary>LP 수립 → 적용 반복 횟수.</summary>
    [Reactive] public partial int LpNumIterations { get; set; } = 40;

    /// <summary>신뢰 영역 기본 단계 (스트립 폭 대비 비율). 막히면 2배씩 확대.</summary>
    [Reactive] public partial float LpTrustRegionRatio { get; set; } = 0.005f;

    /// <summary>좌측으로 당기는 목적항 가중치.</summary>
    [Reactive] public partial float LpLeftPullWeight { get; set; } = 0.02f;

    #endregion

    #region Initial Placement (N06)

    /// <summary>N06 초기배치 페이즈 사용 여부. 켜면 LBF 초기해를 N06 엔진 결과로 교체한 뒤 기존 파이프라인을 잇는다.</summary>
    [Reactive] public partial bool InitEnabled { get; set; } = false;

    /// <summary>초기배치 시간 예산(초).</summary>
    [Reactive] public partial float InitTimeLimitSecs { get; set; } = 60f;

    /// <summary>GRASP 구성 반복 수.</summary>
    [Reactive] public partial int InitGrasp { get; set; } = 1;

    /// <summary>kick(step 재확장 재시도) 횟수. 0 = kick 없음.</summary>
    [Reactive] public partial int InitKicks { get; set; } = 100_000;

    /// <summary>축소 시작 step 비율.</summary>
    [Reactive] public partial float InitStep0Frac { get; set; } = 0.005f;

    /// <summary>축소 최소 step 비율.</summary>
    [Reactive] public partial float InitMinFrac { get; set; } = 0.0005f;

    /// <summary>분리 반복 상한.</summary>
    [Reactive] public partial int InitIterCap { get; set; } = 3000;

    /// <summary>분리 이동 표본 수 기저.</summary>
    [Reactive] public partial int InitSamplesBase { get; set; } = 50;

    /// <summary>GLS 가중치 증가율.</summary>
    [Reactive] public partial float InitGrowth { get; set; } = 1.1f;

    /// <summary>rescue(전수 재배치) 발동 주기. 0 = 끔.</summary>
    [Reactive] public partial int InitRescueAfter { get; set; } = 300;

    /// <summary>정체 기반 종료 무개선 상한. 0 = 반복 수 종료.</summary>
    [Reactive] public partial int InitStallCap { get; set; } = 1500;

    /// <summary>좌표 하강 최소 스텝(mm).</summary>
    [Reactive] public partial float InitCdMin { get; set; } = 0.03f;

    /// <summary>슬라이드 재압축 패스 상한. 0 = 끔.</summary>
    [Reactive] public partial int InitCompactPasses { get; set; } = 6;

    /// <summary>reseat 대상 조각 수. 0 = 끔.</summary>
    [Reactive] public partial int InitReseatTop { get; set; } = 4;

    /// <summary>배치 후보 평가 상한.</summary>
    [Reactive] public partial int InitCandEvalCap { get; set; } = 32;

    /// <summary>좌하단 점수 X 가중치.</summary>
    [Reactive] public partial float InitLbfXWeight { get; set; } = 500f;

    /// <summary>좌하단 점수 Y 가중치.</summary>
    [Reactive] public partial float InitLbfYWeight { get; set; } = 0.1f;

    #endregion

    #region NFP-LBF 초기 배치 / 적응형 탈출

    /// <summary>
    /// NFP 기반 초기 배치(NFP-LBF)를 탐색 seed 후보로 만든다. <b>기본 켜짐.</b>
    /// <para>
    /// 켜면 Init 이 LBF 배치와 NFP-LBF 배치를 둘 다 만들고, 각각에 짧은 탐색(조각 수에 따라
    /// 1 ~ 3초)을 돌려 더 좁아진 쪽을 고른다(«시드 대결»). 초기 폭만으로 고르면 양쪽으로 다
    /// 틀리기 때문이다 — 네이티브 <c>seed_duel_prefers_candidate</c> 문서 참조. 시드를 못 만들거나
    /// 초기 폭이 3% 넘게 넓으면 대결 없이 LBF 를 쓴다.
    /// </para>
    /// <para>
    /// 2026-10-02 실측(3코어, 탐색 480초): babel 겉감 200조각(한 방향) 82.7% → 시드 5개 평균
    /// 85.7%(84.9 ~ 86.2%). 여성WSjk 156 은 차이 없음(81.0% 대 81.1 / 80.9%). 표준 인스턴스 7종
    /// (탐색 90초)도 ±0.1%p 안. 대가는 Init 시간이다 — 시드 탐색(최대
    /// <see cref="NfpLbfTotalBudgetMs"/>)과 대결(2 ~ 6초)만큼 길어진다.
    /// </para>
    /// </summary>
    [Reactive] public partial bool NfpLbfEnabled { get; set; } = true;

    /// <summary>평가할 ordering 수. 0 이하면 코어 기본값(50).</summary>
    [Reactive] public partial int NfpLbfOrderings { get; set; } = 50;

    /// <summary>NFP 콜드 캐시 생성 예산(ms). 0 이하면 코어 기본값(5000).</summary>
    [Reactive] public partial int NfpLbfCacheBudgetMs { get; set; } = 5000;

    /// <summary>초기 배치 탐색 전체 예산(ms). 0 이하면 코어 기본값(10000).</summary>
    [Reactive] public partial int NfpLbfTotalBudgetMs { get; set; } = 10000;

    /// <summary>0/180도 회전을 함께 쓴다. 결방향 없는(non-nap) 원단 전용.</summary>
    [Reactive] public partial bool NfpLbfAllow180 { get; set; }

    /// <summary>
    /// 아이템 하나당 feasible 판정을 시도할 raw 후보 수 상한. 0 이하면 코어 기본값(4096).
    /// <para>
    /// <b>초기 배치 품질을 좌우하는 값이다.</b> 실측(여성WSjk 156-패널, LBF = 69.45%):
    /// 1024 → 68.02%(LBF 에 져서 시드가 기각된다), 2048 → 69.90%,
    /// 4096 → 71.83%(기본값), 8192 → 72.08%. 전체 시간은 8초 → 14초.
    /// </para>
    /// </summary>
    [Reactive] public partial int NfpLbfMaxProbes { get; set; } = 4096;

    /// <summary>
    /// NFP 라이브러리 전용 오목 단순화 강도. 0 이면 비활성.
    /// 실측: 0.02 에서 캐시 생성 2677ms → 162ms, 밀도 손해 없음.
    /// </summary>
    [Reactive] public partial float NfpLbfConcaveSimplify { get; set; } = 0.02f;

    /// <summary>정체 감지 기반 적응형 국소 최적해 탈출로 탐색 단계를 교체한다.</summary>
    [Reactive] public partial bool EscapeEnabled { get; set; }

    /// <summary>조각 사이 겹침 허용(mm). Draft Init에도 적용. 0이면 비활성.</summary>
    [Reactive] public partial float PieceOverlapMm { get; set; } = 0.08f;

    /// <summary>
    /// 원본 외곽선 겹침이 겹침 허용값을 넘는 배치의 처리. 0 = 실현 불가능(기본, 버림),
    /// 1 = 실현 가능(채택). 네이티브 로그 <c>[ORIGINAL:VALIDATE] original overlap exceeds
    /// configured piece_overlap_mm</c> 가 나오는 배치가 대상이다. 겹침 허용이 0 이면 효과가 없다.
    /// </summary>
    [Reactive] public partial int OriginalOverlapPolicyIndex { get; set; }

    /// <summary>
    /// V2 place groups(패턴 그룹) 처리 방식. 0 = 다중 폴리곤 아이템(기본), 1 = 강체 이동(1.0.2 방식).
    /// </summary>
    [Reactive] public partial int PlaceGroupModeIndex { get; set; }

    /// <summary>탐색 단계를 순수 NFP 순서 탐색으로 교체한다. 적응형 탈출보다 우선한다.</summary>
    [Reactive] public partial bool NfpOrderingExploration { get; set; }

    /// <summary>① separator 위치 샘플러를 NFP 후보로 교체한다.</summary>
    [Reactive] public partial bool NfpSampler { get; set; }

    /// <summary>② Conflict Burst 분리 벡터를 정확한 NFP 기반으로 바꾼다.</summary>
    [Reactive] public partial bool NfpExactSeparationVector { get; set; }

    #endregion

    #region NFP 초기배치 전략 7종 / LP·QP 솔버

    /// <summary>
    /// ① 회전 후보를 인스턴스가 선언한 <c>allowed_orientations</c> 에서 그대로 뽑는다.
    /// <para>
    /// 실측(albano, <c>[0,180]</c>): 끄면 LBF 대비 −1.51%p 로 지고, 켜면 +8.57%p 로 이긴다.
    /// 회전 선언이 <c>[0]</c> 뿐인 인스턴스(여성WSjk)는 영향이 없다.
    /// </para>
    /// </summary>
    [Reactive] public partial bool StrategyInstanceRotations { get; set; }

    /// <summary>① 연속 회전 샘플 간격(도). 작을수록 촘촘하지만 NFP 캐시가 제곱으로 커진다.</summary>
    [Reactive] public partial int StrategyRotationStepDeg { get; set; } = 90;

    /// <summary>② 잘 맞물리는 조각 쌍을 하나로 묶어 배치한다. 의류의 좌/우 대칭 패널이 대상.</summary>
    [Reactive] public partial bool StrategyPairwiseClustering { get; set; }

    /// <summary>② 쌍을 채택할 최소 충전율 이득.</summary>
    [Reactive] public partial float StrategyClusterMinGain { get; set; } = 0.05f;

    /// <summary>③ 후보를 "가장 왼쪽" 이 아니라 "접촉 최대" 로 고른다.</summary>
    [Reactive] public partial bool StrategyContactMaximization { get; set; }

    /// <summary>④ 프론티어를 늘리기 전에 내부 구멍부터 채운다.</summary>
    [Reactive] public partial bool StrategyPocketFirst { get; set; }

    /// <summary>⑤ 부분 배치를 1개가 아니라 여러 개 유지한다.</summary>
    [Reactive] public partial bool StrategyBeamSearch { get; set; }

    /// <summary>⑤ 빔 폭. 시간이 이 값에 정비례한다.</summary>
    [Reactive] public partial int StrategyBeamWidth { get; set; } = 2;

    /// <summary>⑥ 순서를 고정하지 않고 매 단계 가장 잘 맞는 조각을 고른다.</summary>
    [Reactive] public partial bool StrategyBestFit { get; set; }

    /// <summary>⑥ 매 단계 평가할 조각 종류 수 상한.</summary>
    [Reactive] public partial int StrategyBestFitLookahead { get; set; } = 12;

    /// <summary>⑦ 배치 후 각 조각을 막힐 때까지 좌하단으로 민다.</summary>
    [Reactive] public partial bool StrategySlidingCompaction { get; set; }

    /// <summary>⑦ 슬라이딩 라운드 수.</summary>
    [Reactive] public partial int StrategySlidingRounds { get; set; } = 2;

    /// <summary>⑧ 순서를 정하지 않고 매 단계 가장 잘 맞물리는 조각을 고른다(누적 배치).</summary>
    [Reactive] public partial bool StrategyAccretion { get; set; }

    /// <summary>⑨ 꼬리의 조각을 빼서 앞쪽 빈 곳으로 되돌린다.</summary>
    [Reactive] public partial bool StrategyTailReflow { get; set; }

    /// <summary>
    /// ⑨ 꼬리 회수 라운드 수. 코어 기본값과 같은 100.
    /// <para>
    /// 라운드 루프는 뺄 개수 일정과 띠 폭 주기를 다 훑으면 <b>스스로 멈추므로</b>
    /// 이 값은 "몇 번 시도할까" 가 아니라 "자기종료를 허용할 상한" 이다.
    /// 실측(wsjk156): 2회 72.06% → 100회 73.53%(여기서 자기종료).
    /// </para>
    /// </summary>
    [Reactive] public partial int StrategyReflowRounds { get; set; } = 100;

    /// <summary>
    /// <b>실험</b> — 레퍼런스 설계 초기배치 엔진. 켜면 기존 초기 배치를 <b>대체</b>한다.
    /// <para>
    /// 구조 검증용이다. 실측(wsjk156): 기존 NFP-LBF 71.75% / 전략 8+7+9 73.72% /
    /// 경쟁사 82.28% / 이 엔진 25.04%.
    /// </para>
    /// </summary>
    [Reactive] public partial bool RefPlaceEnabled { get; set; }
    [Reactive] public partial bool ColumnDraftEnabled { get; set; }

    /// <summary>
    /// 실험 엔진의 그룹 대체 규칙. 0 = 조각 하나가 한 그룹, 1 = item_id 별 묶음(기본).
    /// </summary>
    [Reactive] public partial int RefPlaceGroupFallback { get; set; } = 1;

    /// <summary>
    /// LBF 초기배치가 조각을 훑는 <b>방향</b>.
    /// 0 = 큰 것부터(기본, 현행), 1 = 작은 것부터,
    /// 2 = 중간부터 큰 쪽 먼저, 3 = 중간부터 작은 쪽 먼저.
    /// <para>
    /// <b>기준 크기가 "면적"이 아니다.</b> 정렬 점수는 <c>볼록껍질 면적 × 직경</c> 이라
    /// 오목한 부분이 메워진 면적에 직경이 곱해진다 — 길쭉한 조각이 앞으로 온다.
    /// 이 설정은 그 점수의 정의는 건드리지 않고 <b>순회 방향만</b> 바꾼다.
    /// </para>
    /// <para>
    /// 예외가 둘 있다 — 4(오목도부터)와 5(크기 × 오목도)는 <b>정렬 키 자체</b>를
    /// 방향성 오목도(<c>LBF수정사항.md</c>)로 바꾼다. 왼쪽·아래로 열린 홈을 가진 조각을
    /// 먼저 놓아 뒤 조각이 그 안에 들어가게 한다.
    /// </para>
    /// </summary>
    [Reactive] public partial int LbfOrderIndex { get; set; }

    /// <summary>
    /// <b>홀 우선 부착</b> — 초기 배치에서, 이미 놓인 조각들에 둘러싸인 빈 주머니
    /// (포켓) 안의 자리를 먼저 시도한다.
    /// <para>
    /// 래스터 격자 전수 스캔은 포켓 내부 위치도 만들어 내지만, 기존 LBF 는 좌·하단
    /// 손실이 최소인 후보 하나만 고르므로 x 가 큰 포켓 후보는 언제나 진다.
    /// 켜면 포켓 구간을 먼저 따로 평가한다.
    /// </para>
    /// <para>
    /// 실측(LBF 단독, seed 0): wsjk156 폭 −0.73% / 밀도 +0.52%p (포켓에 18개 부착),
    /// swim −0.09% / +0.06%p, shirts +0.80% / −0.62%p(나빠짐). 기본 꺼짐.
    /// </para>
    /// </summary>
    [Reactive] public partial bool HoleFirstEnabled { get; set; }

    /// <summary>포켓 후보에 내줄 후보 슬롯 비율 (0.0~1.0). 0 이하면 네이티브 기본값 0.5.</summary>
    [Reactive] public partial double HoleFirstShare { get; set; } = 0.5;

    /// <summary>
    /// <b>폴리시 잔여 예산 회수</b> — 폴리시가 예산을 다 쓰기 전에 수렴하면,
    /// 남은 시간 동안 압축↔폴리시를 반복해 더 좁은 폭을 노린다.
    /// <para>
    /// 폴리시는 시계가 아니라 고정점에서 끝난다. 실측(52조각 마커)에서 60초 예산 중
    /// 2.4~7.4초만 쓰고 나머지는 버려진다. 폴리시를 더 돌리는 것은 소득이 없고
    /// (라운드 100·LP 반복 400·정확 NFP 홀채움 모두 폭 동일), 압축 재진입만이
    /// 유일하게 실측된 이득이다.
    /// </para>
    /// <para>
    /// <b>총 소요 시간은 늘지 않는다</b> — 마감이 폴리시 예산의 끝이다.
    /// 실측 4회: −17.11 / +0.00 / +0.00 / −8.00mm. 한 번도 나빠지지 않았다.
    /// </para>
    /// </summary>
    [Reactive] public partial bool ReclaimLeftoverEnabled { get; set; } = true;

    /// <summary>한 사이클에서 압축이 받을 몫 (0.0~1.0). 0 이하면 네이티브 기본값 0.75.</summary>
    [Reactive] public partial double ReclaimCompressShare { get; set; } = 0.75;

    /// <summary>사이클 수 상한. 0 = 예산까지. 재현성이 필요하면 고정한다.</summary>
    [Reactive] public partial int ReclaimMaxCycles { get; set; }

    /// <summary>Reclaim UI selection: 0 = compression reentry, 1 = escape.</summary>
    [Reactive] public partial int ReclaimModeIndex { get; set; }

    /// <summary>
    /// 압축 단계 LP·QP 엔진. 0=microlp(단체법), 1=clarabel(내점법, QP), 2=Clp, 3=Xpress.
    /// <para>Clp/Xpress 는 런타임에 DLL 을 찾는다. 없으면 폴백 설정에 따른다.</para>
    /// </summary>
    [Reactive] public partial int LpBackendIndex { get; set; }

    /// <summary>2차 정규화 계수. 0 이면 순수 LP. QP 백엔드(clarabel, Xpress)에서만 의미가 있다.</summary>
    [Reactive] public partial float LpQuadraticRegularization { get; set; }

    /// <summary>백엔드를 쓸 수 없을 때 microlp 로 물러날지. 끄면 압축을 건너뛴다.</summary>
    [Reactive] public partial bool LpFallback { get; set; } = true;

    /// <summary>한 번의 LP 풀이 제한 시간(ms).</summary>
    [Reactive] public partial int LpTimeLimitMs { get; set; } = 5000;

    #endregion

    #region 초기 구성기 / 원단 경계 (ABI v17)

    /// <summary><b>구간 완성 폭(mm).</b> 0이면 끔.
    /// <para>
    /// 현재 세로 구간이 더 받아들이지 못할 때까지 프론티어를 오른쪽으로 밀지 않는다.
    /// 여성WSjk 156-패널 실측(겹침 0.8mm): 300mm에서 초기 밀도 73.350% → 73.903%.
    /// </para></summary>
    [Reactive] public partial float ConstructorBandWidthMm { get; set; }

    /// <summary>조각의 원본 외곽선이 원단 밖으로 넘어도 되는 거리(mm). 0이면 검사 안 함.
    /// <para>
    /// 켜면 각 단계 결과를 공개하기 직전에 넘은 조각만 안쪽으로 민다. 막는 이웃까지
    /// 같은 방향으로 함께 밀어 겹침 계약은 유지된다.
    /// </para></summary>
    [Reactive] public partial float FabricEdgeMarginMm { get; set; } = 0.2f;

    #endregion

    /// <summary>
    /// 이 저장소의 실측에서 가장 좋았던 값으로 되돌린다.
    /// </summary>
    /// <remarks>
    /// 근거는 모두 여성WSjk 156-패널, 겹침 0.8mm, 3코어 고정 조건의 측정이다.
    /// 다른 마커에서 같은 값이 최선이라는 보장은 없다 — 특히 구간 폭은 조각 폭 분포에
    /// 따라 반응이 단조롭지 않았다(200mm에서 오히려 −3.5%p).
    ///
    /// 여기 넣지 않은 것들은 측정에서 <b>부호가 음수</b>였기 때문이다: 양쪽 모서리
    /// 앵커링, NFP 가중합 우선 선택, 길이 동률 맞물림 선택, 구간 경계 오목부 채우기,
    /// GLS 학습률 조정, 구성 도중 다지기(노이즈 수준인데 시간이 4배).
    /// </remarks>
    public void ApplyRecommended()
    {
        // 구성기 — 미세 래스터가 초기 폭 7128 → 6935mm (+1.99%p).
        RasterEnabled = true;
        RasterCellRatio = 0.02f;
        RasterMaxCells = 4_000_000;
        RasterNumRandomSamples = 4000;
        // 구간 완성 — 위의 래스터 위에서 6935 → 6883mm (+0.55%p).
        ConstructorBandWidthMm = 300f;

        // 원단 경계 — 보정 없이는 조각이 최대 0.35mm 밖으로 나간다(꼭짓각 31.7°).
        FabricEdgeMarginMm = 0.2f;

        // 원단 계약 — 이 저장소의 측정 조건.
        PieceOverlapMm = 0.08f;
    }

    /// <summary>현재 값으로 SwarmConfig를 생성한다.</summary>
    public SwarmConfig ToSwarmConfig() => new()
    {
        RngSeed = RngSeed,
        // 검증은 실행 직전에 한다 — Structure 를 켠 실행은 이 순서를 쓰지 않으므로 비어 있어도 된다.
        PhaseSequence = PhaseEntries.Select(entry => entry.Phase).ToArray(),
        PolySimplTolerance = MmSimplificationEnabled ? null : PolySimplTolerance,
        SimplifyMaxDeviationMm = MmSimplificationEnabled ? SimplifyMaxDeviationMm : null,
        NfpLbf = new NfpLbfSettings
        {
            Enabled = NfpLbfEnabled,
            Orderings = NfpLbfOrderings,
            CacheBudgetMs = NfpLbfCacheBudgetMs,
            TotalBudgetMs = NfpLbfTotalBudgetMs,
            Allow180 = NfpLbfAllow180,
            ConcaveSimplifyTolerance = NfpLbfConcaveSimplify,
            MaxCandidateProbes = NfpLbfMaxProbes,
        },
        Escape = new EscapeSettings { Enabled = EscapeEnabled },
        FabricTolerance = new FabricToleranceSettings { PieceOverlapMm = PieceOverlapMm },
        OriginalOverlapPolicy = OriginalOverlapPolicyIndex == 1
            ? OriginalOverlapPolicy.Feasible
            : OriginalOverlapPolicy.Infeasible,
        PlaceGroupMode = PlaceGroupModeIndex == 1 ? PlaceGroupMode.Rigid : PlaceGroupMode.MultiPolygon,
        FabricEdgeMarginMm = FabricEdgeMarginMm > 0f ? FabricEdgeMarginMm : null,
        Constructor = new ConstructorSettings
        {
            BandWidthMm = ConstructorBandWidthMm,
        },
        NfpIntegration = new NfpIntegrationSettings
        {
            OrderingExploration = NfpOrderingExploration,
            NfpSampler = NfpSampler,
            ExactSeparationVector = NfpExactSeparationVector,
        },
        NfpStrategy = new NfpStrategySettings
        {
            InstanceRotations = StrategyInstanceRotations,
            ContinuousRotationStepDeg = StrategyRotationStepDeg,
            PairwiseClustering = StrategyPairwiseClustering,
            ClusterMinGain = StrategyClusterMinGain,
            ContactMaximization = StrategyContactMaximization,
            PocketFirst = StrategyPocketFirst,
            BeamSearch = StrategyBeamSearch,
            BeamWidth = StrategyBeamWidth,
            BestFitSelection = StrategyBestFit,
            BestFitLookahead = StrategyBestFitLookahead,
            SlidingCompaction = StrategySlidingCompaction,
            SlidingRounds = StrategySlidingRounds,
            Accretion = StrategyAccretion,
            TailReflow = StrategyTailReflow,
            ReflowRounds = StrategyReflowRounds,
        },
        RefPlace = new RefPlaceSettings
        {
            Enabled = RefPlaceEnabled || ColumnDraftEnabled,
            ColumnDraft = ColumnDraftEnabled,
            GroupFallback = RefPlaceGroupFallback,
        },
        // 범위 밖 값은 네이티브가 0(현행 동작)으로 되돌린다.
        LbfOrder = (LbfOrderMode)LbfOrderIndex,
        HoleFirst = HoleFirstEnabled,
        HoleFirstShare = (float)HoleFirstShare,
        ReclaimLeftover = ReclaimLeftoverEnabled,
        ReclaimCompressShare = (float)ReclaimCompressShare,
        ReclaimMaxCycles = ReclaimMaxCycles,
        ReclaimMode = ReclaimModeIndex == 1 ? ReclaimMode.Escape : ReclaimMode.CompressReentry,
        LpSolver = new LpSolverSettings
        {
            // 범위 밖 값은 네이티브가 microlp 로 떨어뜨린다.
            Backend = (LpSolverBackend)LpBackendIndex,
            QuadraticRegularization = LpQuadraticRegularization,
            Fallback = LpFallback,
            TimeLimitMs = LpTimeLimitMs,
        },
        MinItemSeparation = MinItemSeparationEnabled ? MinItemSeparation : null,
        NarrowConcavityCutoffDist = NarrowConcavityCutoffDist,
        NarrowConcavityCutoffArea = NarrowConcavityCutoffArea,
        lbf_x_multiplier = LbfXMultiplier,
        lbf_y_multiplier = LbfYMultiplier,
        Exploration = new ExplorationConfig
        {
            ShrinkStep = ExplShrinkStep,
            TimeLimitSecs = ExplTimeLimitSecs,
            MaxConseqFailedAttempts = ExplMaxConseqEnabled ? ExplMaxConseqFailedAttempts : null,
            SolutionPoolDistributionStdDev = ExplSolutionPoolStdDev,
            LargeItemChAreaCutoffPercentile = ExplLargeItemCutoffPercentile,
            // Garment 기본은 period 0(비활성). 사용자가 켜면 매 3번째 disruption에서 실행한다.
            AdaptiveShrinkGain = AdaptiveShrinkGain,
            AdaptiveShrinkMaxMult = AdaptiveShrinkMaxMult,
            ShrinkFailDecay = ShrinkFailDecay,
            ShrinkScaleMin = ShrinkScaleMin,
            OscFailThreshold = OscFailThreshold,
            OscExpandSteps = OscExpandSteps,
            ReliefFailThreshold = ReliefFailThreshold,
            ReliefStep = ReliefStep,
            ReliefMaxRatio = ReliefMaxRatio,
            ReliefMaxRounds = ReliefMaxRounds,
            ReliefStrikeLimit = ReliefStrikeLimit,
            ReliefRoundBudgetMs = ReliefRoundBudgetMs,
            Separator = new SeparatorConfig
            {
                IterNoImprovementLimit = ExplSepIterNoImprvLimit,
                StrikeLimit = ExplSepStrikeLimit,
                NumWorkers = ExplSepNumWorkers,
                NumContainerSamples = ExplSepNumContainerSamples,
                NumFocussedSamples = ExplSepNumFocussedSamples,
                NumCoordDescents = ExplSepNumCoordDescents,
                LeftBiasGain = ExplSepLeftBiasGain
            }
        },
        Compression = new CompressionConfig
        {
            ShrinkRangeMin = CmprShrinkRangeMin,
            ShrinkRangeMax = CmprShrinkRangeMax,
            TimeLimitSecs = CmprTimeLimitSecs,
            ShrinkDecay = CmprShrinkDecay,
            ShrinkDecayRatio = CmprShrinkDecayRatio,
            Separator = new SeparatorConfig
            {
                IterNoImprovementLimit = CmprSepIterNoImprvLimit,
                StrikeLimit = CmprSepStrikeLimit,
                NumWorkers = CmprSepNumWorkers,
                NumContainerSamples = CmprSepNumContainerSamples,
                NumFocussedSamples = CmprSepNumFocussedSamples,
                NumCoordDescents = CmprSepNumCoordDescents,
                LeftBiasGain = CmprSepLeftBiasGain
            }
        },
        Polish = new PolishConfig
        {
            Enabled = PolishEnabled,
            TimeLimitSecs = PolishTimeLimitSecs,
            NumRounds = PolishNumRounds,

            SlideEnabled = SlideEnabled,
            SlideNumPasses = SlideNumPasses,
            SlideDown = SlideDown,
            SlideSafetyRatio = SlideSafetyRatio,

            LpEnabled = LpEnabled,
            LpNumIterations = LpNumIterations,
            LpTrustRegionRatio = LpTrustRegionRatio,
            LpLeftPullWeight = LpLeftPullWeight
        },
        LbfRaster = new RasterNfpConfig
        {
            Enabled = RasterEnabled,
            CellRatio = RasterCellRatio,
            MaxCells = RasterMaxCells,
            NumCandidates = RasterNumCandidates,
            NumRandomSamples = RasterNumRandomSamples
        },
        StructureFirst = new StructureFirstConfig
        {
            Enabled = StructureFirstEnabled,
            MotifEnabled = StructureMotifEnabled,
            P2TimeLimitSecs = CmprTimeLimitSecs,
            ProgressCallbackIntervalMs = StructureProgressCallbackIntervalMs
        },
        CallbackOptions = new CallbackOptionsConfig
        {
            CollectLoss = CallbackCollectLoss,
            CollectCollisionStatus = CallbackCollectCollisionStatus
        },
        InitialPlacement = new InitialPlacementConfig
        {
            Enabled = InitEnabled,
            TimeLimitSecs = InitTimeLimitSecs,
            Grasp = InitGrasp,
            Kicks = InitKicks,
            Step0Frac = InitStep0Frac,
            MinFrac = InitMinFrac,
            IterCap = InitIterCap,
            SamplesBase = InitSamplesBase,
            Growth = InitGrowth,
            RescueAfter = InitRescueAfter,
            StallCap = InitStallCap,
            CdMin = InitCdMin,
            CompactPasses = InitCompactPasses,
            ReseatTop = InitReseatTop,
            CandEvalCap = InitCandEvalCap,
            LbfXWeight = InitLbfXWeight,
            LbfYWeight = InitLbfYWeight
        },
        Allow180Rotation = Allow180Rotation,
        TiltRangeDeg = TiltRangeDeg
    };
}
