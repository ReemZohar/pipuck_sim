# Project Plan: Biologically Inspired Swarm Network Optimization on Pi-Puck Robots

This plan implements the Di-PL and Soap Bubble Optimization algorithms from `docs/algorithm.pdf` and `docs/algorithm_addons.pdf` in the Pi-Puck ARGoS simulator.

**Current point:** Tasks 0–1, 2.1, 2.2.1, and 2.2.2 are implemented. Complete Task 2.0 before continuing with Task 2.2.3.

---

## Phase 0: Build the Basics and Setup

Verify the build pipeline for a new controller before writing algorithm code.

### Task 0.1: Create Controller Skeleton

- [x] **0.1.1:** Create `CPhybotController` with `Init()`, `ControlStep()`, and `Reset()`.
- [x] **0.1.2:** Register it with `REGISTER_CONTROLLER(CPhybotController, "phybot_controller")`.
- [x] **0.1.3:** Register its sources in CMake.

### Task 0.2: Verify Build and Registration

- [x] **0.2.1:** Build with `./compile_ws.sh` and load the controller from an ARGoS experiment.

---

## Phase 1: Architecture, State, and Configuration

### Task 1.1: Define Data Structures and Serialization

- [x] **1.1.1:** Represent the Di-PL tuple `(relative location, sender pressure, conductivity, flow, timestamp)`.
- [x] **1.1.2:** Serialize and deserialize fields in the same order.
- [x] **1.1.3:** Keep the heavy message and every Pi-Puck `rab_data_size` at 52 bytes.

### Task 1.2: State Storage

- [x] **1.2.1:** Store incoming and outgoing message history for the last `H` ticks and discard expired entries.
- [x] **1.2.2:** Store pressure `p_i`, food/flux `Q_i`, timestamp, role, and eight sector conductivities `D_i,k`.
- [x] **1.2.3:** Store each neighbor message with its locally sensed range, bearing, and sector.
- [ ] **1.2.4:** Add the Algorithm 1 initialization state: `D_init` and capacity `Q_c`.

### Task 1.3: Parameter Loading and XML Configuration

- [x] **1.3.1:** Load the current Di-PL and motion parameters from `<params>`.
- [x] **1.3.2:** Configure `rab_data_size="52"` and `rab_range` in every experiment.
- [ ] **1.3.3:** Load `D_init` and `Q_c`; reject invalid physical/numerical values at `Init()`.
- [ ] **1.3.4:** Before Phase 3, add the remaining paper parameters: `lambdaL`, `alphaC`, `alphaS`, `wd`, and the numerical epsilon.

### Task 1.4: LED Visual Debugging

- [x] **1.4.1:** Retrieve the Pi-Puck LED actuator.
- [x] **1.4.2:** Use green for `SOURCE`, blue for `TARGET`/nest, and white for `NORMAL`.

### Task 1.5: Initialization Tests

- [ ] **1.5.1:** Create `pipuck_ws/tests/` for deterministic C++ tests and a `tests/CMakeLists.txt` for their test executables.
- [ ] **1.5.2:** Configure GoogleTest with `find_package(GTest REQUIRED)` and add the tests subdirectory. Register each executable through `add_phybot_test()` so it is included in the generated test-executable list.
- [ ] **1.5.3:** Create executable `tests/run_tests.sh`. It configures `build/` with CMake, builds the test targets, and runs every executable in the generated list directly; it must return a non-zero status on any failure.
- [ ] **1.5.4:** Add deterministic tests for message round trips and the exact 52-byte payload.
- [ ] **1.5.5:** Test empty history, no receiver, and zero conductivity/range; no result may be non-finite.
- [ ] **1.5.6:** Test `Reset()` restores empty history, zero timestamp/food, `D_init`, and no stale outgoing message.
- [ ] **1.5.7:** Test source/target/normal initialization: `+P_max`, `-P_max`, and zero pressure respectively.

---

## Phase 2: Adaptation Mechanism (Distributed Plasticity Learning — Di-PL)

Implement Algorithm 1 as receive → incoming pressure update → send → outgoing pressure update. The two pressure updates must remain separate.

### Task 2.0: Establish Di-PL Boundary Conditions and RAB Semantics

Complete this task now, before Task 2.2.3.

- [ ] **2.0.1:** Implement local source/target detection. A controller sets its own role from its sensors; loop functions configure the environment and gather metrics but do not give controllers global swarm state.
- [ ] **2.0.2:** On initialization/reset, set normal pressure to zero, source pressure to `+P_max`, target pressure to `-P_max`, every sector conductivity to `D_init`, and every flow to zero.
- [ ] **2.0.3:** Define the RAB adaptation protocol. RAB broadcasts one 52-byte packet, while Algorithm 1 has directed `(D_ij, Q_ij)` messages. Document how the intended sector/receiver is identified, how other receivers ignore the message, and how the receiver obtains edge length from its own range reading.
- [ ] **2.0.4:** Verify the protocol with two neighbors in different sectors: only the intended receiver records the flow as incoming.

### Task 2.1: Implement Pressure Updates (Hagen–Poiseuille)

- [x] **2.1.1:** Implement incoming pressure (Eq. 1): mean sender pressure minus `L_ji Q_ji / D_ji`.
- [x] **2.1.2:** Implement outgoing pressure (Eq. 2): mean receiver pressure plus `L_ij Q_ij / D_ij`.
- [x] **2.1.3:** Implement exponential smoothing and safe pressure decay (Eq. 3).
- [ ] **2.1.4:** Test Eq. 1–3 with known messages, an empty history, and zero conductivity.

### Task 2.2: Implement Flux and Conductivity Updating

- [x] **2.2.1:** Calculate vertex food `Q_i` from incoming flows (Eq. 4).
- [x] **2.2.2:** Select at most one lowest-pressure receiver per sector.
- [ ] **2.2.3:** For each selected receiver, calculate estimated outgoing flow `Qhat_ij = D_ij / L_ij * (p_i - p_j)`; normalize positive estimates so their total equals at most `Q_i` (Eq. 5). Protect all zero denominators.
- [ ] **2.2.4:** Update the selected sector/edge conductivity with Eq. 6. Define the non-negative reinforcement function `f(|Q_ij|)` in the implementation and ensure an unused edge decays.
- [ ] **2.2.5:** Subtract each sent flow from `Q_i`, record the outgoing logical message, and broadcast it using Task 2.0.3's protocol.
- [ ] **2.2.6:** Test flow conservation, used-edge reinforcement, unused-edge decay, and no-receiver behavior.

### Task 2.3: Wire Algorithm 1 into `ControlStep()`

- [ ] **2.3.1:** Process incoming messages and source injection, then update `Q_i` and incoming pressure.
- [ ] **2.3.2:** Select receivers, calculate/send flows, update conductivities, and decrease `Q_i`.
- [ ] **2.3.3:** Calculate outgoing pressure after sending; do not update pinned source/target pressures.
- [ ] **2.3.4:** Expire old messages once per tick and then advance the timestamp.

---

## Milestone 1: Static Flow and Pressure Validation

Before motion, validate Di-PL with motion disabled.

- [ ] **M1.1:** Run a three-robot chain: source → normal → target. Verify message propagation, source > relay > target pressure, and flow conservation.
- [ ] **M1.2:** Add a longer alternative route and verify that its unused conductivity decays relative to the used route.
- [ ] **M1.3:** Reproduce the paper's ring check: `D_init = 1`, `betaD = 10`, 200 ticks, 50 seeded runs; average conductivity must favor the shorter route.

---

## Phase 3: Motion Mechanism (Soap Bubble Optimization and Exploration)

### Task 3.1: Calculate Local Penalties

- [ ] **3.1.1:** Implement epsilon-protected repulsion `Fr_ij = 1 / L_ij^2`.
- [ ] **3.1.2:** Implement angular penalty `P_theta = cos(theta) + 1` from local neighbor bearings.
- [ ] **3.1.3:** Combine them as `Z_i = wp * sum(Fr) + (1 - wp) * sum(P_theta)`.

### Task 3.2: Evaluate Candidate Positions

- [ ] **3.2.1:** Build the local grid with cells of size `2dr` within `dc`; legal moves are unblocked straight-line steps no longer than `ds`.
- [ ] **3.2.2:** Remove candidates blocked by locally sensed obstacles or robots.
- [ ] **3.2.3:** Choose the candidate maximizing `DeltaZ = Z_current - Z_candidate`; remain still when no safe positive gain exists.
- [ ] **3.2.4:** Convert the selected direction to bounded differential-drive velocities.

### Task 3.3: Implement Exploration and Exploitation

- [ ] **3.3.1:** Implement Random-Walk, Wobble, Spread, and circular exploration using only local readings.
- [ ] **3.3.2:** Implement `shouldExplore` from conductivity, neighbor count, and the epsilon-greedy probability in Section 6.1.1.
- [ ] **3.3.3:** Normalize gain and mean conductivity as described in Appendix D, then implement `shouldExploit` from Section 6.1.2.

---

## Phase 4: Swarm Network Optimization (SNO) Integration

### Task 4.1: Implement Algorithm 2

- [ ] **4.1.1:** Execute Phase 2's adaptation cycle, then update sector state, local neighbor/obstacle readings, and food decay `Q_i *= (1 - gammaQ)`.
- [ ] **4.1.2:** Execute one motion decision: explore, or evaluate the best candidate and apply `shouldExploit`.
- [ ] **4.1.3:** Keep `ControlStep()` allocation-free after initialization and run a seeded headless smoke test after integration changes.

---

## Phase 5: Loop Functions and Metrics

### Task 5.1: Sources, Targets, and Dynamics

- [ ] **5.1.1:** Configure static source/target entities and their local controller detection.
- [ ] **5.1.2:** Add moving targets only after static terminals work; verify role changes arise from local sensing.

### Task 5.2: Track Evaluation Metrics

- [ ] **5.2.1:** Per tick, record terminal connectivity, nearest-neighbor distance sum, movement, food delivered, and route conductivity.
- [ ] **5.2.2:** Calculate Chamfer distance only where a GeoSteiner/reference layout exists.
- [ ] **5.2.3:** Include the seed, parameters, scenario, and tick in machine-readable logs; aggregate repeated runs outside controllers.

---

## Phase 6: Validation and Experimentation

### Task 6.1: Minimal Mobile Sanity Check

- [ ] **6.1.1:** Run 3–4 mobile robots with a fixed seed. Verify collision avoidance and a stable source-to-target connection.

### Task 6.2: Ring, Maze, and Adaptability

- [ ] **6.2.1:** Run the ring experiment from M1.3 with motion enabled.
- [ ] **6.2.2:** Run the paper's maze setup for 1,600 ticks and 20 seeded runs using Spread and `wp = 0.375`.
- [ ] **6.2.3:** Test expendable/reappearing sources and verify useful new chains replace dissolved ones.

### Task 6.3: Steiner and Dynamic Scenarios

- [ ] **6.3.1:** Test triangle, square, rectangle, pentagon, and bifocal-triangle cases for 500 ticks across 30 seeded runs.
- [ ] **6.3.2:** Test scattered and nest-start conditions separately.
- [ ] **6.3.3:** Test moving targets and report connection maintenance over time.
- [ ] **6.3.4:** Only then run the large benchmark: 15 terminals, 150 robots, 600 ticks, 20 seeded runs.
