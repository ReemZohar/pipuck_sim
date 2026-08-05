# Agent Configurations & Guidelines

This document outlines the core protocols, coding rules, and reference resources for anyone working in this repository.

---

## Coding Principles

All code implementations in the **[pipuck_ws](pipuck_ws)** workspace must adhere to the following principles:

1. **Strict Local Sensing**: Controllers must operate using *only* local information obtained through sensors. They must never bypass local sensing or access global state (which is reserved exclusively for **[loop_functions/](pipuck_ws/loop_functions)**).
2. **Reset Safety**: The controller's `Reset()` method must completely reinitialize all internal state variables to starting values. Never assume variables are zeroed out across simulation resets.
3. **Zero Dynamic Allocations**: Avoid heap allocations (`new`/`delete`, unreserved `std::vector::push_back`) inside `ControlStep()` to keep the simulation lightweight.
4. **Numerical Stability**: Ensure division-by-zero checks are in place for repulsion forces and vector normalizations using protection epsilons ($\epsilon$).
5. **Serialization Safety**: Ensure `rab_data_size` in the `.argos` files matches the exact serialized message size (52 bytes for `CPhybotHeavyMessage`), and that the serialized field order between the RAB transmitter and receiver matches.
6. **Concise Implementations**: Keep implementations as short and simple as possible. Avoid boilerplate, redundant logic, and over-engineering.
7. **Meaningful Comments Only**: Do not write comments that restate what the code is doing (e.g. `i++; // increment i`). Only add comments to explain the *why* (non-obvious rationale, design decisions, or complex algorithms). Always preserve all existing user comments and docstrings.
8. **Self-Documenting Code**: Use descriptive, intention-revealing names for functions, classes, and variables (e.g. `selectSectorReceivers()`). Names should be long enough to be self-explanatory but short enough to stay readable (see rule 15).
9. **Naming Conventions**: Use `camelCase` for function names and for non-member variables (e.g. `deserializeMsg`, `extractParameters`). Class names use `PascalCase` with a leading capital letter (e.g. `SPhybotMessage`, `CPhybotController`). Member variables follow the ARGoS Hungarian prefix convention — an `m_` prefix followed by a type tag (`f` for `Real`, `pc` for pointers to control-interface classes, `un`/`e`/`s` for unsigned/enum/struct) — e.g. `m_fEstimatedPressure`, `m_pcWheels`, `m_unTimestamp`, `m_eRole`, `m_messageList`.
10. **No Hardcoded Magic Numbers**: Extract constants (such as sensor thresholds, physical offsets, and weights) into config parameter blocks or explicit constant variables instead of hardcoding them in inline calculations.
11. **4-Space Indentation**: Use 4 spaces for indentation in all C++ source files (`.hpp`, `.cpp`, `.h`). Do not use tabs.
12. **Minimal & Targeted Changes**: Only modify the specific code, functions, or files requested by the user. Never alter unrelated code logic, class signatures, or static modifiers, and never delete existing user comments.
13. **Header Includes in .hpp Only**: Always place all `#include` directives (standard library headers, ARGoS headers, custom module headers) in `.hpp` header files. `.cpp` source files must ONLY include their corresponding `.hpp` header file (e.g. `phybot_controller.cpp` should only contain `#include "phybot_controller.hpp"`).
14. **Line Length Limit**: No source line may exceed 130 characters. If a statement would exceed the limit, break it across lines or shorten it where possible.
15. **Reasonable Name Length**: Function and variable names must strike a balance between descriptiveness and brevity. Prefer concise names that fit on a single line and are easy to scan; avoid long compound names when a shorter one conveys the same intent.
16. **SOLID Principles**: Code should follow the SOLID principles — single responsibility, open/closed, Liskov substitution, interface segregation, and dependency inversion — keeping classes focused on one purpose, shallow interfaces, and dependencies on abstractions rather than concrete implementations.



---

## Core Reference Resources

- **Swarm Implementation Details**: Refer to **[simulation_plan.md](pipuck_ws/simulation_plan.md)** for mathematical equations, tasks, and validation milestones.
- **Academic Source Material**:
  - Main paper: **[algorithm.pdf](pipuck_ws/docs/algorithm.pdf)** (Di-PL and SBO model).
  - Supplementary docs: **[algorithm_addons.pdf](pipuck_ws/docs/algorithm_addons.pdf)** (Default parameter tables, appendix proofs).

---

## Agent Workflow Protocols

- **Workspace Boundaries**: All operations and command runs must be scoped strictly within the repository root.