# DimOS Quick Reference

A concise reference for understanding how DimOS works.

## The Big Picture

```
┌─────────────────────────────────────────────────────────────┐
│  You write:                                                 │
│    blueprint = autoconnect(modules...).build()             │
│                                                             │
│  DimOS does:                                                │
│    1. Deploys modules as Dask actors                        │
│    2. Wires streams (In[T] ↔ Out[T]) by name+type           │
│    3. Assigns transports (LCM, SharedMemory, etc.)          │
│    4. Links RPC methods (convention-based)                 │
│    5. Starts everything                                     │
└─────────────────────────────────────────────────────────────┘
```

## Core Abstractions

| Concept | What It Is | Key Point |
|---------|------------|-----------|
| **Module** | Distributed unit of functionality | Runs as Dask actor, has streams, RPC, skills |
| **Stream** | Typed data channel (`In[T]`, `Out[T]`) | Matched by `(name, type)`, uses transports |
| **Blueprint** | Declarative module specification | Describes WHAT, not HOW to wire |
| **Transport** | Message passing abstraction | LCM (network), SharedMemory (local), etc. |
| **Agent** | LLM reasoning system | Discovers and calls skills |
| **Skill** | Agent-callable method | `@skill()` decorator, becomes LLM tool |

## Module Definition Pattern

```python
class MyModule(Module):
    # Declare streams
    input_data: In[DataType] = None
    output_data: Out[DataType] = None
    
    # Declare RPC dependencies
    rpc_calls = ["OtherModule.method"]
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    @rpc
    def start(self):
        # Initialize resources
        self.input_data.subscribe(self.process)
    
    @skill()
    def do_something(self, param: str) -> str:
        """Agent-callable skill."""
        return f"Did {param}"
```

## Blueprint Composition Pattern

```python
# Basic
blueprint = autoconnect(
    ModuleA.blueprint(),
    ModuleB.blueprint(),
)

# With configuration
blueprint = (
    autoconnect(
        ModuleA.blueprint(),
        ModuleB.blueprint(),
    )
    .transports({
        ("image", Image): pSHMTransport("/camera/image"),
    })
    .global_config(n_dask_workers=4)
    .remappings([
        (ModuleA, "color_image", "rgb_image"),
    ])
)

# Build and run
coordinator = blueprint.build()
coordinator.loop()  # Runs until Ctrl+C
```

## Stream Matching Rules

1. **Name + Type must match**: `Out[Image]` named `"image"` connects to `In[Image]` named `"image"`
2. **Remappings applied first**: `color_image` → `rgb_image` before matching
3. **Same transport per group**: All `(name, type)` pairs share one transport instance
4. **Type safety**: `Out[Image]` cannot connect to `In[String]` even if names match

## RPC Wiring Convention

```python
# Module A (provides method)
class ModuleA(Module):
    @rpc
    def get_data(self) -> str:
        return "data"

# Module B (needs method)
class ModuleB(Module):
    rpc_calls = ["ModuleA.get_data"]
    
    @rpc
    def set_ModuleA_get_data(self, rpc_call: RpcCall):
        self._get_data = rpc_call
        self._get_data.set_rpc(self.rpc)
    
    def use_it(self):
        data = self._get_data()  # Remote call
```

**Convention**: `set_<ModuleName>_<method_name>` is called automatically during build.

## Transport Selection

```
Priority:
1. Explicit override in .transports()
2. Auto-select:
   - Has lcm_encode? → LCMTransport
   - No lcm_encode? → pLCMTransport
3. Topic: /{name} or random ID
```

## Agent-Skill Pattern

```python
# Skill module
class NavigationSkills(SkillModule):
    @skill(stream=Stream.call_agent)
    def navigate_to(self, location: str) -> str:
        """Navigate to location."""
        yield "Moving..."  # Update agent
        # ... navigation logic ...
        yield "Arrived"    # Update agent
        return "Complete"

# Agent discovers skills automatically
blueprint = autoconnect(
    NavigationSkills.blueprint(),
    llm_agent(system_prompt="You are a robot."),
)
```

## Common Patterns

### 1. Hardware → Perception → Navigation

```python
blueprint = autoconnect(
    robot_connection(),      # Hardware interface
    camera_module(),          # Sensor processing
    object_detector(),        # Perception
    navigation_planner(),     # Planning
    behavior_tree_navigator(), # Execution
)
```

### 2. Adding Agent Control

```python
blueprint = autoconnect(
    basic_robot_stack(),      # Hardware + perception + nav
    navigation_skill(),      # Expose nav as skill
    llm_agent(),             # Reasoning agent
    human_input(),           # CLI interface
)
```

### 3. Performance Optimization

```python
blueprint = autoconnect(
    modules...
).transports({
    # High-frequency camera: shared memory
    ("color_image", Image): pSHMTransport("/camera/image"),
    
    # Low-frequency commands: LCM
    ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
})
```

## Lifecycle

```
1. Define modules (classes)
2. Create blueprints (Module.blueprint())
3. Compose blueprints (autoconnect())
4. Configure (transports, global_config, remappings)
5. Build (blueprint.build())
   ├─> Start Dask cluster
   ├─> Deploy modules
   ├─> Wire streams
   ├─> Link RPC
   └─> Start modules
6. Run (coordinator.loop())
7. Stop (Ctrl+C or coordinator.stop())
```

## Key Files

| File | Purpose |
|------|---------|
| `dimos/core/module.py` | Module base class, lifecycle, RPC |
| `dimos/core/blueprints.py` | Blueprint system, autoconnect, build |
| `dimos/core/module_coordinator.py` | Dask deployment, lifecycle |
| `dimos/core/stream.py` | Stream classes, transport interface |
| `dimos/core/transport.py` | Transport implementations |
| `dimos/agents2/agent.py` | LLM agent implementation |
| `dimos/protocol/skill/skill.py` | Skill decorator, execution |

## Debugging Tips

1. **Check stream connections**: `module.io()` shows inputs/outputs
2. **Verify RPC wiring**: Check `_bound_rpc_calls` dict
3. **Inspect transports**: Check transport assignments in blueprint
4. **Monitor skills**: Use `skillspy` to watch skill execution
5. **Check Dask workers**: ModuleCoordinator shows worker assignments

## Common Gotchas

1. **Stream names must match** (or use remappings)
2. **Stream types must match** (type safety enforced)
3. **RPC methods need `@rpc` decorator**
4. **Skills need `@skill()` decorator** (not `@rpc`)
5. **Passive skills alone won't keep agent alive** (need active skill)
6. **Transport selection**: Check `lcm_encode` support for auto-selection




