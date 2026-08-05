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

## Scope & Editing Directives

- **Targeted Modifications Only**: Never perform wholesale refactoring or modify files/methods outside the explicit user request.
- **Preserve Comments & Logic**: Keep all existing user comments, method signatures, static modifiers, and existing logic intact.
- **Selective Commenting**: Add meaningful comments for newly introduced logic without adding redundant comments.

---

## Naming Conventions

Matches the ARGoS plugin style (see AGENTS.md rule 9):

- **Functions / non-member variables**: `camelCase` (e.g. `extractParameters`, `getLowestPressureNeighborPerSector`).
- **Classes**: `PascalCase` with a leading `C` (controllers, entities) or `S` (structs): `CPhybotController`, `CPhybotMessage`, `SNeighborReading`.
- **Member variables**: `m_` prefix + type tag:
  - `f` → `Real` (`m_fAlpha`, `m_fEstimatedPressure`)
  - `pc` → pointer to control-interface class (`m_pcWheels`, `m_pcRABSens`)
  - `un` → unsigned (`m_unTimestamp`, `m_unH`)
  - `e` → enum (`m_eRole`)
  - `s` → struct (`m_messageList`)

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

### `rab_data_size` must match serialized message size

`CPhybotHeavyMessage` serializes 4 `Real` + 1 `uint32_t` via `CByteArray` operators. ARGoS encodes each `double` as an `SInt64` mantissa + `SInt32` exponent (**12 bytes**, see `byte_array.cpp::operator<<(double)`), so the wire size is `4 * 12 + 4 = 52` bytes. The XML must have `rab_data_size="52"` on each `<pipuck>` entity, and `CRABEquippedEntity::SetData` throws if a transmitted message size mismatches:

```xml
<pipuck id="pipuck0" led_medium="leds" rab_data_size="52" rab_range="1.5">
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
<pipuck id="pipuck0" led_medium="leds" rab_data_size="52" rab_range="1.5">
  <body position="0.0,0.0,0"/>
  <controller config="phybot_controller"/>
</pipuck>
```

The `config` attribute value must match the controller `id`.

---

## ARGoS Logging

- **`LOG`**: Always prints to console. Use for debugging output.
- **`RLOG`**: Per-robot logger — does NOT appear in console output, but is viewable in the simulator GUI (per-robot log window). Use it for per-robot debug messages that you want to inspect live in the simulation. Also filterable via `-log <entity_id>`.
- **`Real` (double) cannot stream to `ostream` directly in some ARGoS builds**: Cast to `float` before logging: `static_cast<float>(value)`.

```cpp
LOG << "[Role] " << GetId() << ": NORMAL" << std::endl;
RLOG << "pressure=" << static_cast<float>(inMsg.m_fSenderEstPressure) << std::endl;
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

### `CPhybotMessage` (`ds/message/phybot_message.hpp`)

Abstract base class with 5 fields:

```cpp
class CPhybotMessage {
public:
    Real m_fRelativeLocation;    // Distance to sender (set by receiver from RAB range)
    Real m_fSenderEstPressure;   // Estimated pressure p_j
    Real m_fEdgeConductivity;    // Conductivity D_ji
    Real m_fEdgeFlow;            // Flow Q_ji
    uint32_t m_unTimestamp;      // Time step t

    virtual CByteArray serialize() const = 0;
    virtual void deserialize(CByteArray& msgBytes) = 0;
};
```

### `CPhybotHeavyMessage` (`ds/message/phybot_heavy_message.hpp`)

Concrete implementation using `Real` (double) for all fields. Serializes via `CByteArray <<`/`>>` operators.

### `CPhybotMessageList` (`ds/message/phybot_message_list.hpp`)

Manages incoming and outgoing message deques with sliding-window expiry based on timestamp.

### `SNeighborReading` (`ds/neighbor_reading.hpp`)

Wraps an incoming RAB message with spatial metadata:

```cpp
struct SNeighborReading {
    std::unique_ptr<CPhybotMessage> msg;
    Real range;          // Distance from RAB sensor
    CRadians bearing;    // Horizontal bearing angle
    u_int8_t sector;     // Sector index (0-7) derived from bearing
};
```

### `ERobotRole` (`algorithms/robot_role.hpp`)

```cpp
enum class ERobotRole { NORMAL, SOURCE, TARGET };
```

---

## File Layout

```
pipuck_ws/
├── CMakeLists.txt              # Top-level build
├── compile_ws.sh               # Build script (cmake + make)
├── algorithms/
│   ├── di_pl.hpp               # Di-PL algorithm class declaration
│   ├── di_pl.cpp               # Di-PL algorithm implementation
│   └── robot_role.hpp          # ERobotRole enum class
├── ds/
│   ├── neighbor_reading.hpp    # SNeighborReading struct
│   └── message/
│       ├── phybot_message.hpp      # Abstract message interface
│       ├── phybot_heavy_message.hpp # Heavy message declaration
│       ├── phybot_heavy_message.cpp # Heavy message serialization
│       ├── phybot_message_list.hpp  # Message buffer manager declaration
│       └── phybot_message_list.cpp  # Message buffer manager implementation
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
- [x] Task 1.1: `CPhybotMessage` class + `CPhybotHeavyMessage` serialization
- [x] Task 1.2: State storage (message history with sliding window, sector conductivities, roles)
- [x] Task 1.3: Parameter loading from XML + `phybot_test.argos` configured
- [x] Task 1.4: LED visual debugging (`updateLEDs()` color-codes roles)

Phase 2 is partially complete:
- [x] Pressure estimation (incoming/outgoing)
- [x] Exponential smoothing update
- [x] Safe decay function `g()`
- [x] Sector-based lowest-pressure neighbor selection
- [ ] `calcOutgoingEstFlux`, `calcOutgoingFlux`, `updateConductivity` (not yet implemented)
- [ ] Wire Di-PL updates into `ControlStep()`
- [ ] Populate outgoing messages with real data

**Next**: Complete Phase 2 (Di-PL flux/conductivity updates) and Milestone 1 (static flow validation).
