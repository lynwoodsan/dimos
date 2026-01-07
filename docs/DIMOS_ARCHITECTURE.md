# DimOS Architecture Overview

This document provides a comprehensive explanation of how DimOS operates and how its components are wired together.

## Table of Contents

1. [High-Level Architecture](#high-level-architecture)
2. [Core Concepts](#core-concepts)
3. [How Components Are Wired](#how-components-are-wired)
4. [System Flow](#system-flow)
5. [Key Components Deep Dive](#key-components-deep-dive)

---

## High-Level Architecture

DimOS is a **distributed, modular framework for building AI-native generalist robots**. The system is built on several key principles:

### Design Philosophy

1. **Modularity**: Every component is a `Module` - hardware drivers, perception algorithms, navigation planners, AI agents
2. **Declarative Composition**: Use blueprints to declare what you want, not how to wire it
3. **Distributed Execution**: Modules run as Dask actors across processes/workers
4. **Type-Safe Communication**: Streams are typed (`In[T]`, `Out[T]`) and validated at build time
5. **Transport Abstraction**: Same code works with local shared memory, network protocols, or distributed clusters

### System Layers

```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                     │
│  (Blueprints: autoconnect(modules...).build())          │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                    Blueprint System                      │
│  (ModuleBlueprintSet: wiring, transport selection, RPC)   │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                    Module Coordinator                    │
│  (Deployment, lifecycle, Dask cluster management)       │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                    Module Layer                          │
│  (Dask actors: streams, RPC, skills, event loops)       │
└─────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                    Transport Layer                       │
│  (LCM, SharedMemory, Zenoh: pub/sub messaging)          │
└─────────────────────────────────────────────────────────┘
```

---

## Core Concepts

### 1. Modules

**What they are**: Distributed, communicating units of functionality - the fundamental building block.

**Key characteristics**:
- Inherit from `Module` (which is `DaskModule`, extending `ModuleBase`)
- Run as Dask actors in separate processes
- Have their own event loop for async operations
- Declare typed streams (`In[T]`, `Out[T]`) for data flow
- Expose RPC methods (via `@rpc` decorator) for synchronous calls
- Can expose skills (via `@skill` decorator) for AI agents

**Example**:
```python
class CameraModule(Module):
    color_image: Out[Image] = None  # Output stream
    
    @rpc
    def start(self):
        # Initialize camera hardware
        pass
```

### 2. Streams

**What they are**: Typed channels for reactive, push-based data flow between modules.

**Types**:
- `In[T]`: Input stream (subscribes to data)
- `Out[T]`: Output stream (publishes data)

**How they work**:
- Built on ReactiveX (RxPY) for reactive programming
- Streams with matching `(name, type)` are automatically connected
- Use transports (LCM, SharedMemory, etc.) for actual message passing
- Support backpressure (drops intermediate values, keeps latest)

**Example**:
```python
class PerceptionModule(Module):
    color_image: In[Image] = None  # Input stream
    
    def start(self):
        self.color_image.subscribe(self.process_image)
    
    def process_image(self, img: Image):
        # Process incoming images
        pass
```

### 3. Blueprints

**What they are**: Declarative specifications for wiring modules together.

**Key components**:
- `ModuleBlueprint`: Specification for a single module
- `ModuleBlueprintSet`: Container for multiple blueprints with configuration
- `autoconnect()`: Function to combine blueprints

**What blueprints do**:
1. Match streams by `(name, type)` and assign shared transports
2. Wire RPC dependencies (modules declare `rpc_calls = ["OtherModule.method"]`)
3. Select appropriate transports (LCMTransport, pLCMTransport, etc.)
4. Handle remappings when stream names don't match
5. Merge global configuration

**Example**:
```python
blueprint = autoconnect(
    CameraModule.blueprint(),
    PerceptionModule.blueprint(),
).transports({
    ("color_image", Image): pSHMTransport("/camera/image")
})
```

### 4. Transports

**What they are**: Abstraction layer for message passing between modules.

**Available transports**:
- **Network**: `LCMTransport`, `pLCMTransport`, `JpegLcmTransport`, `ZenohTransport`
- **Local**: `pSHMTransport`, `SHMTransport`, `JpegShmTransport`

**Selection logic**:
1. Explicit override in `.transports()` configuration
2. Auto-select: `LCMTransport` if type has `lcm_encode`, else `pLCMTransport`
3. Topic naming: `/{name}` if unique, else random ID

**Key property**: Streams with identical `(remapped_name, type)` share the same transport instance, enabling pub/sub.

### 5. Agents

**What they are**: LLM-based reasoning systems that orchestrate robot behavior.

**How they work**:
- Discover skills from modules (via `@skill` decorator)
- Convert skills to LLM tool schemas (using docstrings as descriptions)
- Run event-driven reasoning loop:
  1. Invoke LLM with conversation history and skill state
  2. Execute tool calls (skills)
  3. Wait for updates from skills
  4. Process results and repeat

**Skill discovery**:
- Modules subclass `SkillModule` (or implement `set_LlmAgent_register_skills`)
- Convention: `set_<ModuleName>_<method>` methods are called during blueprint build
- Skills are registered with the agent automatically

### 6. Skills

**What they are**: Methods on modules that become tools agents can call.

**Key features**:
- Decorated with `@skill()`
- Docstring becomes tool description for LLM
- Can stream updates to agent (`Stream.call_agent` or `Stream.passive`)
- Can use reducers for backpressure (`Reducer.latest`, etc.)

**Example**:
```python
class NavigationSkills(SkillModule):
    @skill(stream=Stream.call_agent)
    def navigate_to(self, location: str) -> str:
        """Navigate to a named location like 'kitchen'."""
        yield "Moving to kitchen..."
        # ... navigation logic ...
        yield "Arrived at kitchen"
        return "Navigation complete"
```

---

## How Components Are Wired

### 1. Stream Wiring

**Process**:
1. Blueprint system extracts `In[T]` and `Out[T]` annotations from modules
2. Groups connections by `(remapped_name, type)` tuple
3. Assigns one transport instance per group
4. All connections in a group share the same transport (enables pub/sub)

**Matching rules**:
- Must match both name AND type
- Remappings applied before matching
- Type safety: `Out[Image]` can only connect to `In[Image]`

**Example flow**:
```python
# Module A
class Producer(Module):
    image: Out[Image] = None

# Module B  
class Consumer(Module):
    image: In[Image] = None

# Blueprint system:
# 1. Extracts: Producer has Out[Image] named "image"
# 2. Extracts: Consumer has In[Image] named "image"
# 3. Matches: (name="image", type=Image) matches
# 4. Creates: Single LCMTransport("/image", Image)
# 5. Assigns: Same transport to both Producer.image and Consumer.image
```

### 2. RPC Wiring

**Process**:
1. Modules declare dependencies: `rpc_calls = ["OtherModule.method"]`
2. Blueprint system finds matching modules during build
3. Creates `RpcCall` objects that proxy to remote methods
4. Calls convention-based setters: `set_<ModuleName>_<method>(rpc_call)`
5. Modules store RPC proxies in `_bound_rpc_calls` dict

**Convention-based wiring**:
```python
class ModuleA(Module):
    @rpc
    def get_data(self) -> str:
        return "data"

class ModuleB(Module):
    rpc_calls = ["ModuleA.get_data"]
    
    @rpc
    def set_ModuleA_get_data(self, rpc_call: RpcCall):
        self._get_data = rpc_call
        self._get_data.set_rpc(self.rpc)
    
    def use_data(self):
        data = self._get_data()  # Calls ModuleA.get_data remotely
```

**Note**: `SkillModule` provides `set_LlmAgent_register_skills` automatically.

### 3. Transport Assignment

**Process**:
1. Check explicit override in `transport_map`
2. If not found, auto-select based on type:
   - Has `lcm_encode`? → `LCMTransport`
   - No `lcm_encode`? → `pLCMTransport`
3. Generate topic name: `/{name}` or random ID

**Transport sharing**:
- All connections with same `(name, type)` share transport instance
- Enables one-to-many pub/sub (one producer, many consumers)
- Reduces memory and connection overhead

### 4. Module Deployment

**Process** (via `ModuleCoordinator`):
1. Start Dask cluster with N workers
2. For each blueprint:
   - Submit module class to Dask as actor
   - Module initializes on worker (creates event loop, RPC server)
   - Returns `RPCClient` proxy
3. Assign transports to streams
4. Connect streams (inputs subscribe to outputs via transport)
5. Wire RPC methods (call convention-based setters)
6. Call `start()` on all modules

**Lifecycle**:
```
INIT → DEPLOY → CONNECT → START → RUNNING → STOP
```

---

## System Flow

### From Blueprint to Running System

```
1. Define Modules
   └─> class MyModule(Module):
         image: Out[Image] = None
         @rpc def start(self): ...

2. Create Blueprints
   └─> blueprint = autoconnect(
         ModuleA.blueprint(),
         ModuleB.blueprint(),
       ).transports({...})

3. Build Blueprint
   └─> coordinator = blueprint.build()
       ├─> Creates ModuleCoordinator
       ├─> Starts Dask cluster
       ├─> Deploys modules as actors
       ├─> Matches and wires streams
       ├─> Assigns transports
       ├─> Wires RPC methods
       └─> Starts all modules

4. Run System
   └─> coordinator.loop()  # Blocks until Ctrl+C
       └─> coordinator.stop()  # Clean shutdown
```

### Data Flow Example

```
CameraModule (Worker 1)
    │
    │ publishes Image
    │ via pSHMTransport("/color_image")
    │
    ▼
[Shared Memory Transport]
    │
    │ broadcasts to all subscribers
    │
    ├─> PerceptionModule (Worker 2)
    │   └─> processes image
    │
    └─> SpatialMemory (Worker 3)
        └─> stores in memory
```

### Agent-Skill Interaction

```
1. Agent receives query: "Go to kitchen"
   │
   ├─> LLM reasons: "I should call navigate_to('kitchen')"
   │
   ├─> Agent calls skill: navigation_skill.navigate_to('kitchen')
   │
   ├─> Skill executes:
   │   ├─> yield "Moving to kitchen..." → agent notified
   │   ├─> Calls RPC: navigation_module.set_goal(...)
   │   ├─> Waits for navigation to complete
   │   └─> yield "Arrived" → agent notified
   │
   └─> Agent receives result, continues reasoning
```

---

## Key Components Deep Dive

### ModuleBase / DaskModule

**Responsibilities**:
- Lifecycle management (init, start, stop)
- Event loop creation and management
- RPC server setup and handling
- Stream discovery (via type annotations)
- Skill hosting (via `SkillContainer`)
- Serialization for Dask deployment

**Key attributes**:
- `_loop`: AsyncIO event loop
- `_rpc`: RPC transport instance
- `_bound_rpc_calls`: Dictionary of bound RPC methods
- `rpc_calls`: Declared RPC dependencies (class attribute)

### ModuleCoordinator

**Responsibilities**:
- Dask cluster lifecycle (start, stop)
- Module deployment as Dask actors
- Module registry (one instance per class)
- Orchestrating build process

**Key methods**:
- `start()`: Start Dask cluster
- `deploy()`: Deploy module class as actor
- `start_all_modules()`: Call start() on all modules
- `loop()`: Run until interrupted, then stop
- `stop()`: Shutdown all modules and cluster

### Blueprint System

**ModuleBlueprint**:
- Immutable specification for a module
- Contains: module class, connections, init args/kwargs

**ModuleBlueprintSet**:
- Container for multiple blueprints
- Builder methods: `.transports()`, `.global_config()`, `.remappings()`
- `build()`: Transforms blueprint into running system

**autoconnect()**:
- Combines multiple `ModuleBlueprintSet` instances
- Deduplicates modules (last wins)
- Merges configuration (last wins)

**Build process** (`blueprint.build()`):
1. Create `ModuleCoordinator` with global config
2. Start Dask cluster
3. Deploy all modules
4. Group connections by `(remapped_name, type)`
5. Assign transports to connection groups
6. Connect streams (inputs subscribe to outputs)
7. Wire RPC methods (call convention-based setters)
8. Start all modules
9. Return coordinator

### Transport System

**Base Transport**:
- Abstract interface: `broadcast()`, `publish()`, `subscribe()`
- Observable integration (ReactiveX)
- Backpressure handling (latest-value semantics)

**PubSubTransport**:
- Topic-based publish-subscribe
- Lazy initialization (resources allocated on first use)
- Error isolation (subscriber errors don't affect others)

**Concrete Transports**:
- `LCMTransport`: UDP multicast, typed messages
- `pLCMTransport`: UDP multicast, pickled Python objects
- `pSHMTransport`: Shared memory, pickled objects (local only)
- `JpegLcmTransport`: Compressed images over network
- `JpegShmTransport`: Compressed images in shared memory

### Agent System

**LlmAgent**:
- LLM-based reasoning module
- Discovers skills from registered modules
- Converts skills to tool schemas
- Runs event-driven reasoning loop

**Agent Loop**:
1. Invoke LLM with conversation + tool definitions
2. Parse tool calls from LLM response
3. Execute skills via `SkillCoordinator`
4. Wait for skill updates (`await coordinator.wait_for_updates()`)
5. Process results and add to conversation
6. Repeat until active skills complete

**Skill Registration**:
- Convention: `set_LlmAgent_register_skills(rpc_call)`
- Called during blueprint build
- Skills converted to tool schemas (function signature + docstring)
- Registered with agent's tool registry

### Skill System

**Skill Decorator**:
- `@skill()`: Marks method as agent-callable
- Parameters:
  - `stream`: `Stream.call_agent` (active) or `Stream.passive` (background)
  - `output`: Output type for streaming
  - `reducer`: How to handle backpressure (`Reducer.latest`, etc.)
  - `hide_skill`: Don't expose as tool (for passive streams)

**Skill Execution**:
- Skills run asynchronously (don't block agent)
- Can yield intermediate updates
- Return value sent to agent when complete
- Streamed updates sent via `SkillCoordinator`

**SkillCoordinator**:
- Tracks skill execution state
- Manages skill updates and results
- Provides `wait_for_updates()` for agent loop

---

## Summary

DimOS operates as a **declarative, distributed robotics framework**:

1. **Define modules** with typed streams and RPC methods
2. **Compose blueprints** declaratively using `autoconnect()`
3. **Build system** automatically wires streams, assigns transports, links RPC
4. **Deploy modules** as Dask actors across workers
5. **Run system** with agents orchestrating via skills

The key insight is **separation of concerns**:
- **Modules** define WHAT they do (streams, RPC, skills)
- **Blueprints** define HOW they're wired (transports, connections)
- **Coordinator** handles WHERE they run (Dask deployment)
- **Transports** handle HOW data moves (LCM, SharedMemory, etc.)

This architecture enables:
- **Modularity**: Components are independent and reusable
- **Composability**: Build complex systems from simple modules
- **Flexibility**: Same code works in dev, test, and production
- **Safety**: Process isolation prevents cascading failures
- **Scalability**: Distribute across multiple workers/machines




