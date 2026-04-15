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

"""Tests for NativeModule rebuild-on-change integration."""

from __future__ import annotations

from pathlib import Path
import stat

import pytest

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.utils.change_detect import PathEntry


@pytest.fixture(autouse=True)
def _use_tmp_cache(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Redirect the change-detection cache to a temp dir for every test."""
    monkeypatch.setattr(
        "dimos.utils.change_detect._get_cache_dir",
        lambda: tmp_path / "cache",
    )


@pytest.fixture()
def build_env(tmp_path: Path) -> dict[str, Path]:
    """Set up a temp directory with a source file, executable path, and marker path."""
    src = tmp_path / "src"
    src.mkdir()
    (src / "main.c").write_text("int main() { return 0; }")

    exe = tmp_path / "mybin"
    marker = tmp_path / "build_ran.marker"

    # Build script: create the executable and a marker file
    build_script = tmp_path / "build.sh"
    build_script.write_text(f"#!/bin/sh\ntouch {exe}\nchmod +x {exe}\ntouch {marker}\n")
    build_script.chmod(build_script.stat().st_mode | stat.S_IEXEC)

    return {"src": src, "exe": exe, "marker": marker, "build_script": build_script}


class _RebuildConfig(NativeModuleConfig):
    executable: str = ""
    rebuild_on_change: list[PathEntry] | None = None


class _RebuildModule(NativeModule):
    config: _RebuildConfig


def _make_module(build_env: dict[str, Path]) -> _RebuildModule:
    """Create a _RebuildModule pointing at the temp build env."""
    return _RebuildModule(
        executable=str(build_env["exe"]),
        build_command=f"sh {build_env['build_script']}",
        rebuild_on_change=[str(build_env["src"])],
        cwd=str(build_env["src"]),
    )


def test_rebuild_on_change_triggers_build(build_env: dict[str, Path]) -> None:
    """When source files change, the build_command should re-run."""
    mod = _make_module(build_env)
    try:
        exe = build_env["exe"]
        marker = build_env["marker"]

        # First build: exe doesn't exist → build runs
        mod._maybe_build()
        assert exe.exists()
        assert marker.exists()
        marker.unlink()

        # No change → build should NOT run
        mod._maybe_build()
        assert not marker.exists()

        # Modify source → build SHOULD run
        (build_env["src"] / "main.c").write_text("int main() { return 1; }")
        mod._maybe_build()
        assert marker.exists(), "Build should have re-run after source change"
    finally:
        mod.stop()


def test_no_change_skips_rebuild(build_env: dict[str, Path]) -> None:
    """When sources haven't changed, build_command must not run again."""
    mod = _make_module(build_env)
    try:
        marker = build_env["marker"]

        # Initial build
        mod._maybe_build()
        assert marker.exists()
        marker.unlink()

        # Second call — nothing changed
        mod._maybe_build()
        assert not marker.exists(), "Build should have been skipped (no source changes)"
    finally:
        mod.stop()


def test_rebuild_when_build_command_changes(build_env: dict[str, Path]) -> None:
    """Changing build_command (e.g. nix tag bump) should trigger a rebuild."""
    mod = _make_module(build_env)
    try:
        exe = build_env["exe"]
        marker = build_env["marker"]

        # Initial build
        mod._maybe_build()
        assert exe.exists()
        marker.unlink()

        # No change → skip
        mod._maybe_build()
        assert not marker.exists()

        # Change build_command (simulates a nix tag bump)
        mod.config.build_command = f"sh {build_env['build_script']}  # v0.2.0"
        mod._maybe_build()
        assert marker.exists(), "Build should re-run when build_command changes"
    finally:
        mod.stop()


def test_rebuild_on_change_none_skips_check(build_env: dict[str, Path]) -> None:
    """When rebuild_on_change is None, no change detection happens at all."""
    exe = build_env["exe"]
    marker = build_env["marker"]

    mod = _RebuildModule(
        executable=str(exe),
        build_command=f"sh {build_env['build_script']}",
        rebuild_on_change=None,
        cwd=str(build_env["src"]),
    )
    try:
        # Initial build
        mod._maybe_build()
        assert exe.exists()
        assert marker.exists()
        marker.unlink()

        # Modify source — but rebuild_on_change is None, so no rebuild
        (build_env["src"] / "main.c").write_text("int main() { return 1; }")
        mod._maybe_build()
        assert not marker.exists(), "Should not rebuild when rebuild_on_change is None"
    finally:
        mod.stop()


def test_should_rebuild_true_bypasses_change_check(build_env: dict[str, Path]) -> None:
    """``should_rebuild=True`` forces a rebuild even when ``did_change`` would skip."""
    mod = _make_module(build_env)
    try:
        exe = build_env["exe"]
        marker = build_env["marker"]

        # Initial build seeds the cache so a normal rebuild would now skip.
        mod._maybe_build()
        assert exe.exists()
        assert marker.exists()
        marker.unlink()

        # Sanity check: with should_rebuild=False (default), nothing changed → skip.
        mod._maybe_build()
        assert not marker.exists()

        # Flip the bypass flag — build runs unconditionally.
        mod.config.should_rebuild = True
        mod._maybe_build()
        assert marker.exists(), "should_rebuild=True must force a rebuild"
        marker.unlink()

        # And it keeps forcing on every call as long as the flag is set.
        mod._maybe_build()
        assert marker.exists(), "should_rebuild=True must force on every call"
    finally:
        mod.stop()


def test_failed_build_does_not_mark_cache_clean(build_env: dict[str, Path]) -> None:
    """A failed ``build_command`` must leave the cache untouched so the next call retries."""
    src = build_env["src"]
    exe = build_env["exe"]

    # Build script that always fails after touching nothing.
    failing = build_env["build_script"].parent / "fail.sh"
    failing.write_text("#!/bin/sh\necho oops >&2\nexit 1\n")
    failing.chmod(failing.stat().st_mode | stat.S_IEXEC)

    mod = _RebuildModule(
        executable=str(exe),
        build_command=f"sh {failing}",
        rebuild_on_change=[str(src)],
        cwd=str(src),
    )
    try:
        # First call: build fails, so the cache must NOT be updated to the
        # current source hash. Otherwise the next call would incorrectly
        # think "nothing changed" and early-return on a stale/missing exe.
        with pytest.raises(RuntimeError, match="Build command failed"):
            mod._maybe_build()

        # Second call without changing sources: should still try to build
        # (and fail again) — proving the cache wasn't poisoned by the first
        # failure.
        with pytest.raises(RuntimeError, match="Build command failed"):
            mod._maybe_build()
    finally:
        mod.stop()
