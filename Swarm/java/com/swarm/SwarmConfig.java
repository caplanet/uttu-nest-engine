package com.swarm;

/**
 * Configuration for Swarm optimizer.
 * Maps to native CSwarmConfig structure.
 */
public class SwarmConfig {
	public long rngSeed = 0; // 0 = random
	public float polySimplTolerance = -1.0f; // negative = None
	public float minItemSeparation = -1.0f;
	public float narrowConcavityCutoffDist = -1.0f;
	public float narrowConcavityCutoffArea = -1.0f;
	/** LBF constructor X-axis placement bias. */
	public float lbfXMultiplier = 10.0f;
	/** LBF constructor Y-axis placement bias. */
	public float lbfYMultiplier = 1.0f;

	public ExplorationConfig exploration = new ExplorationConfig();
	public CompressionConfig compression = new CompressionConfig();
	public PolishConfig polish = new PolishConfig();

	/**
	 * LBF raster NFP starting-placement candidates (capmods builds only).
	 * Off by default - it narrows the LBF starting width but no final-density gain
	 * was measured; enable for placement-hint warm starts or very short time budgets.
	 */
	public RasterNfpConfig lbfRaster = new RasterNfpConfig();

	/**
	 * Add 180-degree flips to each item's allowed rotations.
	 * Non-nap fabrics only. Default false. (+0.8%p density on real markers, 60s)
	 */
	public boolean allow180Rotation = false;

	/**
	 * Small-angle tilt (+/- degrees) allowed around discrete rotations.
	 * 0 = off (default). Keep 0 for nap/patterned fabrics.
	 * (+2.2%p density at +/-2 degrees on real markers, 60s)
	 */
	public float tiltRangeDeg = 0.0f;

	/** Excess original-outline overlap (beyond the piece overlap allowance): reject. Default. */
	public static final int ORIGINAL_OVERLAP_INFEASIBLE = 0;
	/** Excess original-outline overlap: accept the layout (original-outline check off). */
	public static final int ORIGINAL_OVERLAP_FEASIBLE = 1;

	/**
	 * How to treat layouts whose original outlines overlap deeper than the piece
	 * overlap allowance (sharp corners shrink more than edges). 0 = infeasible
	 * (discard, default), 1 = feasible (accept). No effect without piece overlap.
	 */
	public int originalOverlapPolicy = ORIGINAL_OVERLAP_INFEASIBLE;

	/** Place groups become one multipart item each (default). */
	public static final int PLACE_GROUP_MULTIPOLYGON = 0;
	/** Place group members stay separate items moved together (pre-1.1 behaviour). */
	public static final int PLACE_GROUP_RIGID = 1;

	/**
	 * How V2 place groups are handed to the engine. 0 = one multipart item per
	 * group (default; members keep their exact relative poses and other pieces may
	 * use the gaps between members), 1 = rigid member moves.
	 */
	public int placeGroupMode = PLACE_GROUP_MULTIPOLYGON;

	/**
	 * Separator configuration.
	 */
	public static class SeparatorConfig {
		public int iterNoImprvLimit = 300;
		public int strikeLimit = 5;
		public int nWorkers = 4;
		public int nContainerSamples = 100;
		public int nFocussedSamples = 50;
		public int nCoordDescents = 5;

		// -- capmods-only fields (ignored by native libraries built without the capmods feature) --

		/** Swap move period in inner-loop iterations. 0 = off. Default 20. */
		public int swapPeriod = 20;
		/** Swaps attempted per pass. Default 8. */
		public int swapAttempts = 8;
		/** Swap partner area-ratio lower bound (relative to the reference item). Default 0.4. */
		public float swapAreaRatioMin = 0.4f;
		/** Swap partner area-ratio upper bound (relative to the reference item). Default 2.5. */
		public float swapAreaRatioMax = 2.5f;
		/** Max left-bias strength of container-wide samples. 0 = uniform. Default 2.0. */
		public float leftBiasGain = 2.0f;
	}

	/**
	 * Exploration phase configuration.
	 */
	public static class ExplorationConfig {
		public float shrinkStep = 0.002f;
		public float timeLimitSecs = 24.0f;
		public int maxConseqFailedAttempts = 10; // -1 = None
		public float solutionPoolDistributionStdDev = 0.3f;
		public float largeItemChAreaCutoffPercentile = 0.7f;
		public SeparatorConfig separator = new SeparatorConfig();

		// -- capmods-only fields (ignored by native libraries built without the capmods feature) --

		/** Run ruin &amp; recreate every N disruptions. 0 = off. Default 3. */
		public int rnrPeriod = 3;
		/** Ruined vertical band width as a fraction of the strip width (0~1). Default 0.18. */
		public float rnrBandFraction = 0.18f;
		/** Max items ruined at once. Default 40. */
		public int rnrMaxItems = 40;
		/** Density-adaptive shrink gain. 0 = fixed step. Default 4.0. */
		public float adaptiveShrinkGain = 4.0f;
		/**
		 * Adaptive shrink max multiplier over shrinkStep. Default 5.0.
		 * <p>
		 * With the default gain of 4.0 this clamp only binds at {@code density <= 0}, so it
		 * never fires in practice; raise the gain above ~10 before tuning it.
		 */
		public float adaptiveShrinkMaxMult = 5.0f;
		/**
		 * Shrink-step failure feedback: multiplier applied to the step scale on the first
		 * separation failure at a width, earned back when a new feasible width is reached.
		 * The density-based gain above is open loop and never learns that a step was too
		 * large; this closes the loop the way the compression phase already does. Only the
		 * first failure at a width counts. Must be in (0, 1); 1.0 = off (default).
		 */
		public float shrinkFailDecay = 1.0f;
		/** Shrink-step failure feedback: lower bound for the step scale. Default 0.05. */
		public float shrinkScaleMin = 0.05f;
		/** Width oscillation after N consecutive fails at a width. 0 = off. Default 4. */
		public int oscFailThreshold = 4;
		/**
		 * Oscillation expansion in base shrinkStep multiples. Default 3.0.
		 * <p>
		 * The effective expansion is {@code min(shrinkStep * this, headroom / 2)} and the
		 * headroom to the best feasible width is always one shrink step, so a value above
		 * {@code adaptiveShrinkMaxMult / 2} is always clamped away. Use
		 * {@link #reliefFailThreshold} to escape past the best feasible width.
		 */
		public float oscExpandSteps = 3.0f;
		/**
		 * Width relief: after N consecutive separation failures at one width, keep the best
		 * colliding layout and widen the strip in small steps (inserted at the collision hot
		 * spot) until the overlaps clear, then resume shrinking from there. Unlike
		 * {@link #oscFailThreshold} this is not capped by the best feasible width; the
		 * incumbent solution is preserved either way. A value &gt; 0 enables relief and makes
		 * {@code oscFailThreshold} inert. 0 = off (default).
		 */
		public int reliefFailThreshold = 0;
		/** Width relief: per-round expansion as a fraction of the current width. Default 0.002. */
		public float reliefStep = 0.002f;
		/** Width relief: cumulative expansion cap relative to the stuck width. Default 0.03. */
		public float reliefMaxRatio = 0.03f;
		/** Width relief: expansion rounds attempted per relief. Default 12. */
		public int reliefMaxRounds = 12;
		/** Width relief: separator strike limit for one expansion round. Default 1. */
		public int reliefStrikeLimit = 1;
		/**
		 * Width relief: wall-clock budget for one expansion round, in milliseconds.
		 * 0 = unlimited (default). The strike limit alone still lets one strike burn the
		 * phase's whole {@code iterNoImprvLimit}, so cap each round here for second-scale
		 * budgets.
		 */
		public int reliefRoundBudgetMs = 0;
		/** ALNS operator-weight decay (0~1). Default 0.85. */
		public float alnsDecay = 0.85f;
		/** ALNS operator-weight floor. Default 0.1. */
		public float alnsMinWeight = 0.1f;
	}

	/**
	 * Compression phase configuration.
	 */
	public static class CompressionConfig {
		public float shrinkRangeMin = 0.001f;
		public float shrinkRangeMax = 0.00001f;
		public float timeLimitSecs = 6.0f;
		public int shrinkDecayStrategy = 0; // 0 = TimeBased, 1 = FailureBased
		public float shrinkDecayRatio = 0.9f;
		public SeparatorConfig separator = new SeparatorConfig();
	}

	/**
	 * Polish (post-processing) phase configuration.
	 *
	 * <p>Applied to the feasible solution produced by the compression phase.
	 * Three deterministic passes run in sequence, {@code nRounds} times:
	 * <ol>
	 *   <li>slide - push every item left (and optionally down) until it touches</li>
	 *   <li>hole fill - rasterize the layout, find enclosed pockets, move small items into them</li>
	 *   <li>lp - LP compaction over per-item translations and the strip width</li>
	 * </ol>
	 * Every pass verifies collisions before committing a move, so the result is
	 * never worse than the input. Frozen items are never moved.
	 */
	public static class PolishConfig {
		public boolean enabled = true;
		public float timeLimitSecs = 5.0f;
		public int nRounds = 3;

		public boolean slideEnabled = true;
		public int slideNPasses = 3;
		public boolean slideDown = true;
		/** Clearance left on contact, relative to the item's shortest side. */
		public float slideSafetyRatio = 0.0f;

		public boolean holeFillEnabled = true;
		/** Grid cell size, relative to the smallest item's shortest side. */
		public float holeFillCellSizeRatio = 0.25f;
		public int holeFillMaxCandidates = 64;
		/** Fraction of the strip's right side considered for relocation (0.0 - 1.0). */
		public float holeFillCandidateBand = 0.25f;
		public int holeFillNSamples = 200;

		public boolean lpEnabled = true;
		public int lpNIterations = 40;
		/** Base (finest) trust-region step, relative to the strip width; grown x2 whenever progress stalls. */
		public float lpTrustRegionRatio = 0.005f;
		/** Weight of the "pull left" objective term. */
		public float lpLeftPullWeight = 0.02f;
	}

	/**
	 * LBF raster NFP starting-placement candidates (capmods builds only).
	 * The layout and the item are rasterized onto the same grid so collision-free
	 * positions can be enumerated directly; every candidate is still verified by the
	 * exact collision engine, so grid resolution never affects correctness.
	 */
	public static class RasterNfpConfig {
		/** Off by default: reliably narrows the LBF starting width, but exploration re-arranges the layout anyway. */
		public boolean enabled = false;
		/** Cell = (smallest item's shortest side) x this ratio. */
		public float cellRatio = 0.12f;
		/** Grid cell cap; the cell grows to satisfy it. */
		public int maxCells = 250_000;
		/** Raster candidates handed to the exact collision engine (all rotations combined). */
		public int nCandidates = 64;
		/** Random samples kept alongside; 0 = raster only. Reducing these to "replace" random search hurts - keep them. */
		public int nRandomSamples = 1000;
	}
}
