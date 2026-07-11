# Agent Configurations & Guidelines

This document outlines the core protocols, coding rules, and reference resources for anyone working in this repository.

---

## Coding Principles

All code implementations in the **[pipuck_ws](pipuck_ws)** workspace must adhere to the following principles:

1. **Strict Local Sensing**: Controllers must operate using *only* local information obtained through sensors. They must never bypass local sensing or access global state (which is reserved exclusively for **[loop_functions/](pipuck_ws/loop_functions)**).
2. **Reset Safety**: The controller's `Reset()` method must completely reinitialize all internal state variables to starting values. Never assume variables are zeroed out across simulation resets.
3. **Zero Dynamic Allocations**: Avoid heap allocations (`new`/`delete`, unreserved `std::vector::push_back`) inside `ControlStep()` to keep the simulation lightweight.
4. **Numerical Stability**: Ensure division-by-zero checks are in place for repulsion forces and vector normalizations using protection epsilons ($\epsilon$).
5. **Serialization Safety**: Ensure packet struct packing/alignment maps exactly between the RAB transmitter and receiver, and verify that `rab_data_size` in the `.argos` files matches the message struct size (16 bytes).
6. **Concise Implementations**: Keep implementations as short and simple as possible. Avoid boilerplate, redundant logic, and over-engineering.
7. **Meaningful Comments Only**: Do not write comments that restate what the code is doing (e.g. `i++; // increment i`). Only add comments to explain the *why* (non-obvious rationale, design decisions, or complex algorithms).
8. **Self-Documenting Code**: Use descriptive, intention-revealing names for functions, classes, and variables (e.g. `GetLowestPressureNeighbor()`) to make the code self-explanatory and reduce comment noise.
9. **No Hardcoded Magic Numbers**: Extract constants (such as sensor thresholds, physical offsets, and weights) into config parameter blocks or explicit constant variables instead of hardcoding them in inline calculations.

---

## Core Reference Resources

- **Swarm Implementation Details**: Refer to **[simulation_plan.md](pipuck_ws/simulation_plan.md)** for mathematical equations, tasks, and validation milestones.
- **Academic Source Material**:
  - Main paper: **[algorithm.pdf](pipuck_ws/docs/algorithm.pdf)** (Di-PL and SBO model).
  - Supplementary docs: **[algorithm_addons.pdf](pipuck_ws/docs/algorithm_addons.pdf)** (Default parameter tables, appendix proofs).

---

## Agent Workflow Protocols

- **Workspace Boundaries**: All operations and command runs must be scoped strictly within the repository root.