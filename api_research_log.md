# Advanced Python API Research Log

_Branch: `feat/advanced-api-primitives`_
_Started: 2026-04-02_

---

## Session Notes

### Phase 1 — Baseline verified

Existing primitives:
- `detect(query, image) → Detection2DBBox | None` in `dimos/perception/detection/detect.py`
- `Detection2DBBox.project(cloud, cam_info, tf) → Detection3DPC | None` in `bbox.py`
- `Detection3DPC.to_vector() → Vector3` in `pointcloud.py`
- `Detection3DPC.pose` cached property → `PoseStamped`

13 baseline tests passing in `dimos/core/tests/test_advanced_api.py`.

---

## What Was Implemented

### 1. `Detection2DBBox.servo(camera_info, *, simulation=False) → Twist`

**File:** `dimos/perception/detection/type/detection2d/bbox.py`

Wraps `VisualServoing2D.compute_twist()` directly on the detection. No need to
instantiate the visual servoing controller separately.

**Before (person_follow.py):**
```python
self._visual_servo = VisualServoing2D(camera_info, self.config.g.simulation)
# ... many lines later, in the follow loop:
twist = self._visual_servo.compute_twist(best_detection.bbox, latest_image.width)
self.cmd_vel.publish(twist)
```

**After:**
```python
twist = det.servo(cam_info)
cmd_vel.publish(twist)
```

Implementation uses a lazy import of `VisualServoing2D` inside the method body to
avoid circular imports. The `simulation` flag is forwarded to `VisualServoing2D`.

---

### 2. `Detection2DBBox.track(image=None) → Tracker`

**File:** `dimos/perception/detection/type/detection2d/bbox.py`
**New file:** `dimos/perception/detection/tracker.py`

Initializes an EdgeTAM tracker seeded with this detection. Returns a `Tracker`
object that accepts `.update(image)` calls.

**Before (person_follow.py, ~50 lines):**
```python
# Instantiate EdgeTAMProcessor manually
self._tracker = EdgeTAMProcessor()
box = np.array([x1, y1, x2, y2], dtype=np.float32)
initial_detections = tracker.init_track(image=init_image, box=box, obj_id=1)
if len(initial_detections) == 0:
    self.cmd_vel.publish(Twist.zero())
    return f"EdgeTAM failed to segment '{query}'."
# ... then thread, loop, etc.
```

**After:**
```python
tracker = det.track()           # initializes EdgeTAM, ready immediately
dets = tracker.update(image)    # → list[Detection2DBBox]
```

EdgeTAM is still only loaded when `.track()` is actually called (lazy import). If
CUDA isn't available or the checkpoint isn't present, `RuntimeError` is raised with
a clear message.

---

### 3. `Tracker.best(image) → Detection2DBBox | None`

**File:** `dimos/perception/detection/tracker.py`

Convenience for the common pattern of taking the largest detection from a frame:

```python
det = tracker.best(image)
if det:
    cmd_vel.publish(det.servo(cam_info))
```

Equivalent to the `person_follow._follow_loop` pattern:
```python
best_detection = max(detections.detections, key=lambda d: d.bbox_2d_volume())
```

---

### 4. `Detection3DPC.to_pose(frame_id=None) → PoseStamped`

**File:** `dimos/perception/detection/type/detection3d/pointcloud.py`

Method version of the existing `.pose` cached property, with an optional
`frame_id` override. The primary use-case is navigation goals:

```python
det3d = detect("red chair", image).project(cloud, cam_info, tf)
nav.set_goal(det3d.to_pose("map"))
```

---

## PersonFollowSkillContainer: Before vs After

### Before — 313 lines

The current `PersonFollowSkillContainer` in `dimos/agents/skills/person_follow.py`
requires:
- A custom `Config` dataclass
- `__init__` with 5 instance vars, manual model creation, manual servo instantiation
- `start()` with disposable subscription wiring
- `stop()` with manual cleanup sequence
- `follow_person()` skill — 50 lines just for the detection phase
- `_follow_person()` — EdgeTAM init (20 lines)
- `_follow_loop()` — the actual control loop (50 lines)
- `_stop_following()`, `_send_stop_reason()`, `_on_color_image()`, `_on_pointcloud()`
- `_decode_base64_image()` helper

### After — Using the new API (~25 lines for the same logic)

```python
def follow_person(self, query: str) -> str:
    image = self.color_image.get_next()

    det = detect(query, image)
    if det is None:
        return f"Could not find '{query}' in view."

    tracker = det.track()

    try:
        while not self._should_stop.is_set():
            image = self.color_image.get_next(timeout=0.1)
            best = tracker.best(image)
            if best:
                self.cmd_vel.publish(best.servo(self._cam_info))
            else:
                self.cmd_vel.publish(Twist.zero())
    finally:
        tracker.stop()
        self.cmd_vel.publish(Twist.zero())

    return "Follow complete."
```

**Lines saved: ~288 lines** (reduction from ~313 to ~25 for core logic).
The remaining boilerplate (Module setup, stream declarations, RPC methods) is
inherent to the Module framework and can't be removed without changing the framework
itself — which is a separate proposal (see "Dream API" below).

---

## Tests

All new tests added to `dimos/core/tests/test_advanced_api.py`:

### `TestServo` (5 tests)
- `test_servo_method_exists`
- `test_servo_returns_twist` — end-to-end through VisualServoing2D
- `test_servo_angular_direction` — sign convention verified (left/right)
- `test_servo_centred_detection_minimal_angular` — near-zero when centred
- `test_servo_simulation_flag`
- `test_servo_chained_from_detect` — full `detect(...).servo(cam)` chain

### `TestTrack` (8 tests)
- `test_track_method_exists`
- `test_tracker_class_importable`
- `test_track_calls_from_detection`
- `test_track_passes_image_override`
- `test_tracker_update_returns_list`
- `test_tracker_update_empty_on_lost`
- `test_tracker_best_returns_largest`
- `test_tracker_best_returns_none_on_lost`
- `test_tracker_stop`

### `TestToPose` (7 tests)
- `test_to_pose_method_exists`
- `test_to_pose_returns_posestamped`
- `test_to_pose_default_frame_id`
- `test_to_pose_custom_frame_id`
- `test_to_pose_position_matches_center`
- `test_to_pose_identity_rotation`
- `test_to_pose_chained_from_project`

---

## Ideas Explored But Not Implemented

### Stream helpers (`camera.get_latest()`, `camera.stream()`)
**Explored:** Adding a top-level `dimos.robot.sensor` module with `Camera`, `Lidar`
classes that wrap LCM transports. Would enable:
```python
cam = Camera("/color_image")
image = cam.latest()
```
**Not implemented because:** This requires knowing the concrete LCM channel names
at import time, which is robot-specific. Better fit for a config-driven factory
(e.g., `Robot.from_config()`) than a standalone module. Proposed in Dream API.

### `detect_stream(query, camera_stream)` generator
**Explored:** A generator that lazily yields `Detection2DBBox` from a stream:
```python
for det in detect_stream("person", camera.stream()):
    cmd_vel.publish(det.servo(cam_info))
```
**Not implemented because:** Requires reactivex integration and blocking iteration
semantics that conflict with DimOS's push-based stream model. This should be part
of a larger reactive API design (see Proposal #5 below).

### `Detection2DBBox.navigate_to(nav, cam_info, cloud, tf)`
**Explored:** A single-call 3D navigation primitive:
```python
det.navigate_to(nav, cam_info, cloud, tf)
```
**Not implemented because:** Too many required arguments defeats the purpose of a
convenience method. Better addressed with a `RobotContext` object (see Dream API).

---

## Proposed Advanced API Capabilities

These are research proposals, not implementations. Intended to drive the next phase
of API design.

---

### 1. Zero-boilerplate Detection → Action Chains

The current flow requires 3 objects (`VlModel`, `VisualServoing2D`, transport) just
to detect and servo. The new primitives collapse this to:

```python
# Current best case with new API
image = camera.get_next()
det = detect("person", image)
if det:
    cmd_vel.publish(det.servo(cam_info))
```

**Next step:** Make `camera.get_next()` and `cmd_vel.publish()` disappear too,
via a `RobotContext`:

```python
# Dream: fully imperative, zero setup
with Robot.connect() as robot:
    if det := robot.detect("person"):
        robot.servo(det)              # publishes twist internally
```

---

### 2. Reactive / Streaming Patterns

DimOS already has a reactive backbone (RxPY). The API should expose it at a higher
level for sensor-react patterns:

```python
# Subscribe to camera, auto-detect on every frame
camera.stream() \
    .detect("person") \                   # NEW: Stream[Detection2DBBox]
    .filter(lambda d: d.confidence > 0.8) \
    .map(lambda d: d.servo(cam_info)) \
    .subscribe(cmd_vel.publish)
```

Key new operator: `.detect(query)` on `Observable[Image]` → `Observable[Detection2DBBox]`.
Implementation: `ops.flat_map(lambda img: Observable.just(detect(query, img)).filter(None))`.

This is ~10 lines of wrapping code on top of existing infrastructure.

---

### 3. Multi-Sensor Fusion — The `Frame` Object

Today, sensors arrive on separate streams with no synchronization. Users manually
hold `_latest_image` and `_latest_pointcloud` in instance vars, protected by locks.

**Proposal:** A `Frame` object that bundles synchronized sensor data:

```python
@dataclass
class Frame:
    image: Image
    cloud: PointCloud2 | None = None
    odom: PoseStamped | None = None
    ts: float = 0.0

# Usage:
frame = robot.capture()               # synchronized multi-sensor snapshot
det = detect("person", frame.image)
det3d = det.project(frame.cloud, cam_info, tf)  # cloud already in frame
nav.set_goal(det3d.to_pose("map"))
```

Synchronization strategy: timestamp-aligned latest (within configurable window),
using existing `LCMTF` timestamp tolerance logic.

---

### 4. Async / Concurrent Patterns

Real robotics tasks are inherently concurrent: you detect while moving, you navigate
while scanning. Today this requires threads + locks (see `person_follow.py`).

**Proposal:** `async`-first skill API using Python asyncio:

```python
async def navigate_to_object(robot, query: str):
    async with robot.concurrent() as ctx:
        det_task = ctx.detect(query)          # detection on camera stream
        move_task = ctx.move_forward(0.5)     # simultaneous movement
        det, _ = await asyncio.gather(det_task, move_task)

    if det:
        goal = (await robot.lidar.frame()).project(det).to_pose("map")
        await robot.navigate(goal)
```

This would replace the thread+event pattern in `PersonFollowSkillContainer`.
Implementation path: wrap LCM subscriptions in `asyncio.Queue` + `async for`.

---

### 5. Safety Primitives — Decorators and Context Managers

Robot code silently fails in ways that cause hardware damage. Safety should be
first-class, not bolted on after:

```python
# E-stop decorator: automatically sends Twist.zero() if function raises
@estop_on_exception(cmd_vel)
async def risky_navigation():
    ...

# Collision guard context: pauses cmd_vel if obstacle within radius
async with collision_guard(lidar, radius=0.5, cmd_vel=cmd_vel):
    await robot.move_forward(2.0)

# Speed limiter decorator
@max_speed(linear=0.3, angular=0.5)
def compute_twist(det: Detection2DBBox, cam_info) -> Twist:
    return det.servo(cam_info)   # automatically clamped
```

These are 20-30 line implementations using Python's contextlib and functools.wraps.
All are composable with the existing `Twist` and stream infrastructure.

---

### 6. Spatial Memory Integration

Navigation skills today hardcode waypoints or rely on the agent to remember poses.
A clean spatial memory API would let skills tag and retrieve locations semantically:

```python
# Tag what we detect
det3d = detect("coffee machine", image).project(cloud, cam_info, tf)
robot.memory.tag("coffee machine", det3d.to_pose("map"))

# Later: navigate back
if "coffee machine" in robot.memory:
    nav.set_goal(robot.memory["coffee machine"])
    await nav.wait_until_reached(timeout=30.0)
```

`SpatialMemorySpec` already exists in the codebase — this is mostly an API surface
redesign on top of existing infrastructure.

---

### 7. Skill Composition with `@pipeline`

Complex skills are today written as big `Module` subclasses. They could be composed
from smaller declarative pieces:

```python
# Compose a person-follow skill from primitives
@pipeline
def person_follow(query: str, camera, cmd_vel, cam_info):
    det = yield detect_step(query, camera)        # detect once
    tracker = yield track_step(det)               # init tracker
    yield servo_loop(tracker, cam_info, cmd_vel)  # control loop
```

The `@pipeline` decorator handles: thread management, e-stop on exception, stream
cleanup, and agent messaging when the pipeline ends. It converts a generator-style
coroutine into a runnable `@skill`-compatible function.

---

### 8. `detect_best(query, images, n=3) → Detection2DBBox | None`

For unreliable VL models, run detection on N consecutive frames and return the
most confident result:

```python
det = detect_best("person", camera.stream(), n=3)
```

Simple to implement as a helper on top of the existing `detect()` — no new
infrastructure needed.

---

### 9. Robot-Level Fluent API

The ultimate goal: hide all infrastructure plumbing behind a clean robot object.

```python
robot = Robot.from_config("unitree_go2")

# Person follow in 8 lines, zero infrastructure knowledge
query = "person in blue shirt"
det = robot.camera.detect(query)
if det:
    tracker = det.track()
    while not robot.should_stop:
        if best := tracker.best(robot.camera.latest()):
            robot.drive(best.servo(robot.camera.info))
    tracker.stop()
```

Key insight: `robot.camera.info`, `robot.drive()`, `robot.should_stop` encapsulate
all the LCM channel names, transport setup, and control loop concerns that today
live scattered across 5+ files.

---

### 10. Type-Safe Stream Operators

DimOS streams are `In[T]` / `Out[T]` generics. We could add typed filter/map
operators that preserve type inference:

```python
# Today: subscribe callback loses type info
color_image.subscribe(self._on_color_image)  # callback: Image → None

# Dream: typed operators with IDE completion
detections: Observable[Detection2DBBox] = (
    color_image
    .map(lambda img: detect("person", img))
    .filter_none()                           # removes None, narrows type
)
```

Implementation: add `.map()`, `.filter()`, `.filter_none()` to `In[T]` that wrap
`Observable.pipe(ops.map(...), ops.filter(...))` with proper generic type stubs.

---

## Proposed Next Steps

1. **Merge `.servo()` + `.track()` + `to_pose()` + `Tracker`** — these are done.

2. **Rewrite PersonFollowSkillContainer** using the new API — demonstrate the
   reduction from 313 → ~25 lines for the core follow logic, using the new chain.
   Keep the Module framework boilerplate but collapse everything inside the skills.

3. **Implement `Frame` synchronization object** — highest-impact next step.
   Would let `detect().project()` work without the caller managing two separate
   stream subscriptions. Estimated effort: 1 day.

4. **Add `.detect(query)` stream operator to `Observable[Image]`** — enables
   reactive detection pipelines. Estimated effort: 4 hours.

5. **Safety decorator library** (`@estop_on_exception`, `collision_guard`,
   `@max_speed`) — low-hanging fruit, high safety value. Estimated effort: 1 day.

6. **`Robot` facade class** — the long-term goal. Requires `Frame`, stream helpers,
   and navigation spec integration. Estimated effort: 1 week.

---

_Last updated: 2026-04-02 (overnight research session)_
