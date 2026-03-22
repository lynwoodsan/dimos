# PR #1431 (Docker Restoration) — Paul Review Fixes

## Commits (local, not pushed)

### 1. `317c487a2` — Include stdout/stderr in docker pull error
- Pull failures were silent — no diagnostic output
- Now includes both stdout and stderr in exception
- **Revert:** `git revert 317c487a2`

### 2. `91a13f1e7` — Import ExceptionGroup in test file
- Test used ExceptionGroup without import → NameError on Python < 3.11
- Now imports from safe_thread_map polyfill
- **Revert:** `git revert 91a13f1e7`

## Reviewer was wrong on
- `rpc_timeouts` class-level mutable dict — it's in ModuleConfig (pydantic) with `Field(default_factory=...)`, which is correct

## Not addressed (need Jeff's input / bigger refactor)
- Container launch in `__init__` vs `start()` — lifecycle redesign
- Deterministic container naming (removed PID+timestamp) — collision risk
- `docker_gpus` default None (was "all") — intentional breaking change?
- `docker_restart_policy` default "no" (was "on-failure:3") — same
- Build hash includes original Dockerfile, not converted (with footer)
- `getattr(default_config, "rpc_timeouts", ...)` returns FieldInfo on class
