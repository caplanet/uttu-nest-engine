# Explore-only DLL benchmark

Build from the repository root:

```powershell
dotnet build scripts/explore_bench/ExploreBench.csproj -c Release --nologo
```

Run paired measurements (replace the absolute candidate DLL path):

```powershell
python scripts/explore_bench/paired.py --baseline C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/pgo-build-20260910-141519/final/capswarmcore_100.dll --candidate C:/absolute/candidate/capswarmcore_100.dll --seeds 17 29 43 --seconds 20 60
```

The default markers are WSjk `C:/Caplanet/AiWorks/InputData/여성WSjk_비교용-G_New_SwarmV1.json` and `target/phase-improvements-bench/inputs/ops2.json`. `--marker name=absolute/path.json` overrides them; repeat the argument for more markers. `--repeats` controls repeated samples. Run without other CPU-heavy tests/builds/benchmarks. The driver starts one process at a time, alternates AB/BA, and holds a driver lock; it cannot prevent unrelated workloads. The default output directory is below the path in `target/explore-priorities-current.txt`; `--out` must name a new directory.

The native DLL loads from the explicit absolute path. ABI version, sizes, features, and required exports are checked through the shared `SwarmNative.cs`. Config starts with native defaults, then uses overlap 2.2 mm, polygon simplification 0.01, three Explore workers, the requested seed/time, and callback loss/collision diagnostics. Only Init and Explore execute; their timings are separate. Each pair must have exactly matching Init widths, pose checksums, full config, ABI, and input hash. There is no post-Init reseed API; Init uses the chosen nonzero seed for both DLLs, including seed 17 by default. No warm hints are synthesized.

Callbacks copy borrowed strings and capture timestamps; parsing and checks run after Explore. Exported native strings are freed. Native callback diagnostics are included in measured Explore time. The harness checks finite poses, IDs/demand, native collision status, feasible callback loss, scalar/JSON width agreement, nonincreasing certified incumbents, and final GetSolution preserving the best observed incumbent. These are native diagnostics under the configured overlap tolerance, not an independent polygon validator. Raw callback payloads and Init/final solution/collision JSON are retained for audit. Width comparisons allow only a small f32 serialization tolerance (`max(1e-5 mm, width * 2e-7)`).

`comparison.json` and CSV give each marker/seed/budget result and strict per-pair width-regression flags. JSON also lists each baseline improvement target and the candidate's first observed attainment time (null means not observed). Throttled callbacks make observed times upper bounds; these are not exact search event timestamps. Small samples do not establish statistical superiority. Exit code 2 means at least one pair regressed in width; process/validation errors stop the run. Completed pairs remain in the report if a later pair fails.

Tiny smoke example (does not measure marker performance):

```powershell
python scripts/explore_bench/paired.py --baseline C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/pgo-build-20260910-141519/final/capswarmcore_100.dll --candidate C:/Caplanet/AiWorks/AdaptiveAlgorithm2/target/pgo-build-20260910-141519/final/capswarmcore_100.dll --marker tiny=C:/Caplanet/AiWorks/AdaptiveAlgorithm2/scripts/explore_bench/tiny.json --seeds 17 --seconds 0.05
```
