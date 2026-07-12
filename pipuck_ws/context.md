# Pi-Puck Swarm Simulation: Context & Developer Guide

This document captures hard-won knowledge about the ARGoS 3 / Pi-Puck build pipeline, XML conventions, and common pitfalls. Read this before touching the code.

---

## Project Overview

We are implementing a biologically inspired swarm network optimization algorithm (Di-PL + Soap Bubble Optimization) on Pi-Puck robots in the ARGoS 3 simulator. The algorithm uses distributed plasticity learning to reinforce shortest paths and soap bubble heuristics for gradient-based motion.

- **Plan**: `simulation_plan.md` — all tasks, equations, and milestones.
- **Paper**: `docs/algorithm.pdf` (Di-PL and SBO model).
- **Supplementary**: `docs/algorithm_addons.pdf` (default parameter tables, proofs).

---

## Build & Run

```bash
# From pipuck_ws/
./compile_ws.sh          # cmake + make, outputs to build/lib/

# Run simulation (GUI)
argos3 -c experiments/phybot_test.argos

# Run simulation (headless / CI / no display)
QT_QPA_PLATFORM=offscreen argos3 -c experiments/phybot_test.argos
```

---

## ARGoS XML Conventions (Critical)

### Controller `<params>` node is passed directly to `Init()`

This is the single most important ARGoS quirk. When a controller is initialized:

1. `CControllableEntity` looks for a `<controller config="..."/>` inside the robot entity.
2. If that controller node has **no** `<params>` child (the common case), it calls `SetController(id)`.
3. `SetController` extracts the `<params>` node from the controller definition in `<controllers>` and passes it to `Init()`.

**Result**: `Init(TConfigurationNode& t_tree)` receives the `<params>` node directly — NOT the parent `<phybot_controller>` node.

### Reading parameters from `<params>`

Since `t_tree` IS the `<params>` node, use `GetNode()` to navigate to each child element, then `GetNodeAttribute()` to read its `value` attribute:

```cpp
// CORRECT — t_tree IS <params>, each param is a child element
GetNodeAttribute(GetNode(t_tree, "alpha"), "value", m_fAlpha);
GetNodeAttribute(GetNode(t_tree, "kp"), "value", m_fKp);

// WRONG — this looks for an XML attribute "alpha" on <params> itself
GetNodeAttribute(t_tree, "alpha", m_fAlpha);
```

The corresponding XML structure:
```xml
<params>
  <alpha value="0.1"/>    <!-- child element with "value" attribute -->
  <kp value="1.0"/>
</params>
```

### `rab_data_size` must match `sizeof(SPhybotMessage)`

`SPhybotMessage` is 10 bytes packed (`3x _Float16` = 6 bytes + `uint32_t` = 4 bytes). The XML must have `rab_data_size="10"` on each `<pipuck>` entity:

```xml
<pipuck id="pipuck0" led_medium="leds" rab_data_size="10" rab_range="1.5">
```

### `ds` vs `rab_range` are NOT the same

- **`rab_range`** (1.5m): RAB communication range — how far messages travel.
- **`ds`** (0.3m): Spreading distance — radius of the movement search grid for candidate positions (Task 3.2.1).

### PiPuck LED medium

The `led_medium` attribute on `<pipuck>` should reference a `<led>` medium (not `directional_led`). The `implementation="color"` on `pipuck_leds` actuator works with `<led>` media despite some README documentation suggesting otherwise.

### Entity-controller reference pattern

```xml
<!-- In <controllers> section -->
<phybot_controller library="build/lib/libpipuck_phybot_controller"
                    id="phybot_controller">
  <actuators>...</actuators>
  <sensors>...</sensors>
  <params>...</params>
</phybot_controller>

<!-- In <arena> section -->
<pipuck id="pipuck0" led_medium="leds" rab_data_size="10" rab_range="1.5">
  <body position="0.0,0.0,0"/>
  <controller config="phybot_controller"/>
</pipuck>
```

The `config` attribute value must match the controller `id`.

---

## ARGoS Logging

- **`LOG`**: Always prints to console. Use for debugging output.
- **`RLOG`**: Requires log category filtering (e.g., `-log <entity_id>`). Does NOT appear in default console output. Avoid for debug prints.
- **`_Float16` cannot stream to `ostream`**: Cast to `float` before logging: `static_cast<float>(value)`.

```cpp
LOG << "[Role] " << GetId() << ": NORMAL" << std::endl;
RLOG << "pressure=" << static_cast<float>(inMsg.senderEstPressure) << std::endl;
```

---

## ARGoS Random Number Generation

```cpp
#include <argos3/core/utility/math/rng.h>

// In Init() — create once, store as member
m_pcRNG = CRandom::CreateRNG("argos");

// In ControlStep() — use the stored instance
m_eRole = static_cast<ERobotRole>(m_pcRNG->Uniform(CRange<SInt32>(0, 2)));
```

Note: `CRandom::GetInstance()` does NOT exist. Always use `CRandom::CreateRNG("argos")`.

---

## Data Structures

### `SPhybotMessage` (`common/phybot_message.h`)

```cpp
struct SPhybotMessage {
   _Float16 senderEstPressure;   // Estimated pressure p_j
   _Float16 edgeConductivity;    // Conductivity D_ji
   _Float16 edgeFlow;            // Flow Q_ji
   uint32_t timestamp;           // Time step t
} __attribute__((packed));       // MUST be 10 bytes
```

Wire format only uses `_Float16`. All internal state/params use `Real` (double) — only serialize to `_Float16` on the wire.

### `ERobotRole` (`common/robot_role.h`)

```cpp
enum class ERobotRole { NORMAL, SOURCE, TARGET };
```

---

## File Layout

```
pipuck_ws/
├── CMakeLists.txt              # Top-level build
├── compile_ws.sh               # Build script (cmake + make)
├── common/
│   ├── phybot_message.h        # SPhybotMessage, serialize/deserialize
│   └── robot_role.h            # ERobotRole enum
├── controllers/
│   ├── CMakeLists.txt
│   ├── phybot_controller.hpp   # Controller class declaration
│   └── phybot_controller.cpp   # Controller implementation
├── loop_functions/
│   ├── CMakeLists.txt
│   ├── loop_functions.hpp      # Loop functions (metrics, global state)
│   └── loop_functions.cpp
├── experiments/
│   └── phybot_test.argos       # ARGoS simulation config
└── docs/
    ├── algorithm.pdf           # Main paper
    └── algorithm_addons.pdf    # Supplementary
```

---

## Current Task Status

Phase 1 is complete:
- [x] Task 1.1: `SPhybotMessage` struct + serialization (10 bytes packed)
- [x] Task 1.2: State storage (message history with sliding window, sector conductivities, roles)
- [x] Task 1.3: Parameter loading from XML + `phybot_test.argos` configured
- [x] Task 1.4: LED visual debugging (`updateLEDs()` color-codes roles)

**Next**: Phase 2 (Di-PL pressure/flow updates) and Milestone 1 (static flow validation).
