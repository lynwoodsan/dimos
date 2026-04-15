# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Benchmark NativeModule rebuild-check latency.

Compares the two ways a NativeModule can decide whether its binary is
up to date on ``start()``:

1. ``rebuild_on_change`` — dimos :func:`did_change` hashes a tracked set
   of source files.  Pure local file I/O.
2. ``should_rebuild=True`` — delegates to the module's ``build_command``
   (typically ``nix build .#foo``) and lets it figure out that nothing
   changed.

Run on the target hardware::

    uv run python dimos/core/native_rebuild_perf_test.py

Both modules must already have been built once (so the nix store has the
cached outputs) — otherwise the ``should_rebuild`` column is measuring a
real build, not a no-op check.  The script warns if the executable is
missing and skips the nix measurements for that module.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import statistics
import subprocess
import time

from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.utils.change_detect import did_change

WARMUP_RUNS = 1
STEADY_RUNS = 10


def _time_fn(fn: Callable[[], None]) -> float:
    """Return wall-clock seconds for one invocation of *fn*."""
    t0 = time.perf_counter()
    fn()
    return time.perf_counter() - t0


def _summarize(samples: list[float]) -> dict[str, float]:
    """Return best / median / p95 / mean in milliseconds."""
    samples_ms = [s * 1000.0 for s in samples]
    samples_ms.sort()
    return {
        "best": samples_ms[0],
        "median": statistics.median(samples_ms),
        "p95": samples_ms[min(len(samples_ms) - 1, int(len(samples_ms) * 0.95))],
        "mean": statistics.mean(samples_ms),
    }


def _fmt_row(label: str, stats: dict[str, float] | None, extra: str = "") -> str:
    if stats is None:
        return f"  {label:<38}  {'(skipped)':>12}{extra}"
    return (
        f"  {label:<38}  "
        f"best {stats['best']:9.2f}ms  "
        f"median {stats['median']:9.2f}ms  "
        f"p95 {stats['p95']:9.2f}ms  "
        f"mean {stats['mean']:9.2f}ms"
        f"{extra}"
    )


def bench_did_change(module: object) -> dict[str, float]:
    """Benchmark one warm + STEADY_RUNS did_change calls."""
    # Mirrors the cache-name computation inlined into NativeModule._maybe_build.
    import inspect

    source_file = Path(inspect.getfile(type(module))).resolve()
    cache_name = f"native_{type(module).__name__}_{source_file}"
    cfg = module.config  # type: ignore[attr-defined]

    def check() -> None:
        did_change(
            cache_name,
            cfg.rebuild_on_change,
            cwd=cfg.cwd,
            extra_hash=cfg.build_command,
        )

    # Seed the cache so we're measuring the "hot" hit path.
    check()
    for _ in range(WARMUP_RUNS):
        check()
    samples = [_time_fn(check) for _ in range(STEADY_RUNS)]
    return _summarize(samples)


def bench_nix_build(
    module: object,
) -> tuple[dict[str, float] | None, float | None, str | None]:
    """Benchmark ``build_command`` as a no-op check.

    Returns ``(steady_stats, cold_ms, skip_reason)``.
    ``cold_ms`` is the wall-clock of the first invocation (eval cache cold).
    Steady stats cover WARMUP_RUNS + STEADY_RUNS subsequent invocations.
    """
    cfg = module.config  # type: ignore[attr-defined]
    exe = Path(cfg.executable)
    if not exe.exists():
        return None, None, f"executable not built yet at {exe}"
    if cfg.build_command is None:
        return None, None, "no build_command configured"

    def run_build() -> None:
        subprocess.run(
            cfg.build_command,
            shell=True,
            cwd=cfg.cwd,
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    # Cold run — eval cache likely empty on this invocation of the command.
    cold = _time_fn(run_build)
    for _ in range(WARMUP_RUNS):
        run_build()
    samples = [_time_fn(run_build) for _ in range(STEADY_RUNS)]
    return _summarize(samples), cold * 1000.0, None


def run_one(module_cls: type) -> None:
    print(f"\n── {module_cls.__name__} " + "─" * (78 - len(module_cls.__name__) - 4))
    module = module_cls()  # type: ignore[call-arg]
    cfg = module.config  # type: ignore[attr-defined]
    print(f"  executable:    {cfg.executable}")
    print(f"  build_command: {cfg.build_command}")
    if cfg.rebuild_on_change:
        print(f"  rebuild_on_change: {len(cfg.rebuild_on_change)} entries")
    print()

    if cfg.rebuild_on_change:
        stats = bench_did_change(module)
        print(_fmt_row("rebuild_on_change (did_change)", stats))
    else:
        print(_fmt_row("rebuild_on_change (did_change)", None, "  (not configured)"))

    nix_stats, cold_ms, skip_reason = bench_nix_build(module)
    if skip_reason:
        print(_fmt_row("should_rebuild (build_command)", None, f"  {skip_reason}"))
    else:
        print(_fmt_row("should_rebuild (build_command, warm)", nix_stats))
        if cold_ms is not None:
            print(f"  {'should_rebuild (build_command, cold)':<38}  first-run {cold_ms:9.2f}ms")


def main() -> None:
    print("=" * 80)
    print("NativeModule rebuild-check benchmark")
    print("=" * 80)
    print(f"  warmup runs: {WARMUP_RUNS}")
    print(f"  steady runs: {STEADY_RUNS}")

    for module_cls in (Mid360, FastLio2):
        run_one(module_cls)

    print()


if __name__ == "__main__":
    main()
