# Project Plan: Biologically Inspired Swarm Network Optimization on Pi-Puck Robots

This plan outlines the design, implementation, and testing of the swarm optimization algorithm, utilizing Pi-Puck robots in the ARGoS 3 simulator.

---

## Table of Contents
0. [Phase 0: Build The Basics & Setup](#phase-0-build-scaffolding--setup)
1. [Phase 1: Architecture, State & Configuration](#phase-1-architecture-state--configuration)
2. [Phase 2: Adaptation Mechanism (Distributed Plasticity Learning - Di-PL)](#phase-2-adaptation-mechanism-distributed-plasticity-learning---di-pl)
3. [Milestone 1: Static Flow & Pressure Validation](#milestone-1-static-flow--pressure-validation)
4. [Phase 3: Motion Mechanism (Soap Bubble Optimization & Heuristics)](#phase-3-motion-mechanism-soap-bubble-optimization--heuristics)
5. [Phase 4: Swarm Network Optimization (SNO) Integration](#phase-4-swarm-network-optimization-sno-integration)
6. [Phase 5: Loop Functions & Metrics Tracking](#phase-5-loop-functions--metrics-tracking)
7. [Phase 6: Validation & Experimentation](#phase-6-validation--experimentation)

---

## Phase 0: Build The Basics & Setup
Verify the build pipeline for a new controller before writing algorithm code to avoid debugging cmake/linkage issues later.

### Task 0.1: Create Controller Skeleton
 * **Mini-Task 0.1.1:** Create [phybot_controller.hpp](controllers/phybot_controller.hpp) inheriting from `CCI_Controller` with basic `Init()`, `ControlStep()`, and `Reset()` declarations.
  * `Reset()` — Called when the simulation is restarted. Reinitializes the controller's state to starting values (clear message history, reset pressure to 0, reset food counters, reset roles to NORMAL).
 * **Mini-Task 0.1.2:** Create [phybot_controller.cpp](controllers/phybot_controller.cpp) with empty implementations and register it via `REGISTER_CONTROLLER(CPhybotController, "phybot_controller")`.
 * **Mini-Task 0.1.3:** Register the new controller source files in [CMakeLists.txt](CMakeLists.txt).

### Task 0.2: Verify Build & Registration
* **Mini-Task 0.2.1:** Compile the workspace using `./compile_ws.sh`.

---

## Phase 1: Architecture, State & Configuration
Lay the software foundations for state representation, parameter parsing, and visual debugging.

### Task 1.1: Define Data Structures & Serialization
* **Mini-Task 1.1.1:** Define a packet structure `SPhybotMessage` containing:
  * `SenderPressure` (float): Estimated pressure of the sender $\hat{p}_j$.
  * `EdgeConductivity` (float): Conductivity of the shared channel $D_{ji}$.
  * `EdgeFlow` (float): Flow along the channel $Q_{ji}$.
  * `Timestamp` (uint32_t): Time step counter $t$.
* **Mini-Task 1.1.2:** Write serialization functions to pack `SPhybotMessage` into ARGoS's `CByteArray` and deserialize it back on reception.

### Task 1.2: State Storage
* **Mini-Task 1.2.1:** Create a list to store the message history window $M(i)$ for the last $H$ time units (discarding old messages).
* **Mini-Task 1.2.2:** Define variables for estimated pressure ($\hat{p}_i$), food received ($Q_i$), and robot role (`NORMAL`, `SOURCE`, `SINK`).
* **Mini-Task 1.2.3:** Discretize the environment surrounding the robot into $2n = 8$ sectors (matching the 8 rangefinder directions) to store sector conductivities $D_{i_k}$.

### Task 1.3: Parameter Loading & XML Configuration
* **Mini-Task 1.3.1:** Implement parameter parsing inside `Init()` from the `<params>` node of the `.argos` file:
  * Load Di-PL parameters: $\alpha$, $k_p$, $\gamma_Q$, $\delta t$, $I_0$, $P_{max}$.
  * Load Motion parameters: $w_p$, $\beta_D$, $\alpha_D$, $\epsilon_D$, $I_e$, $\gamma$, $k$, $d_s$, $H$.
* **Mini-Task 1.3.2:** Configure `experiments/phybot_test.argos` with proper settings, ensuring `rab_data_size` matches the size of `SPhybotMessage` and `rab_range` is set.

### Task 1.4: LED Visual Debugging
* **Mini-Task 1.4.1:** Retrieve the `m_pcColoredLEDs` actuator in `Init()`.
* **Mini-Task 1.4.2:** Implement a helper function `UpdateLEDs()` to color-code robots based on their current roles:
  * `SOURCE` (Food Source) = **Green**
  * `SINK` (Nest) = **Blue**
  * `NORMAL` (Chains/Relays) = **White** (or **Off** if they have 0 conductivity/inactive).

---

## Phase 2: Adaptation Mechanism (Distributed Plasticity Learning - Di-PL)
Implement Algorithm 1 from the paper to enable the reinforcement of shortest paths and decay of unused links.

### Task 2.1: Implement Pressure Updates (Hagen-Poiseuille)
* **Mini-Task 2.1.1:** Write the incoming pressure estimation function `CalculateIncomingPressure()` (Eq. 1):
  $$\hat{P}_{in}(i) = \frac{1}{|S(i)|} \sum_{j \in S(i)} \left( \hat{p}_j - \frac{L_{ji} Q_{ji}}{D_{ji}} \right)$$
* **Mini-Task 2.1.2:** Write the outgoing pressure estimation function `CalculateOutgoingPressure()` (Eq. 2):
  $$\hat{P}_{out}(i) = \frac{1}{|R(i)|} \sum_{j \in R(i)} \left( \hat{p}_j + \frac{L_{ij} Q_{ij}}{D_{ij}} \right)$$
* **Mini-Task 2.1.3:** Implement the exponential smoothing update (Eq. 3) to update the robot's pressure $\hat{p}_i$:
  $$\hat{p}_i = (1 - \alpha) \hat{p}_i^{t-1} + \alpha \cdot g(x(t))$$

### Task 2.2: Implement Flux and Conductivity Updating
* **Mini-Task 2.2.1:** Calculate total food $Q_i$ at the vertex by summing all incoming fluxes (Eq. 4).
* **Mini-Task 2.2.2:** Select the receiver (one per sector) with the lowest pressure to maximize flow.
* **Mini-Task 2.2.3:** Calculate estimated outgoing flux $\hat{Q}_{ij}$ and normalize it to conserve flow (Eq. 5).
* **Mini-Task 2.2.4:** Implement the conductivity update rule (Eq. 6) to decay/reinforce the link values:
  $$\frac{D_{ij}^{t+1} - D_{ij}^t}{\delta t} = f(|Q_{ij}^t|) - D_{ij}^{t+1}$$

---

## Milestone 1: Static Flow & Pressure Validation
Before writing complex motion controls, run a sanity check on a static formation.
* **Task:** Position 3 static robots in a line: `Robot 0 (Source) -> Robot 1 (Normal) -> Robot 2 (Sink)`.
* **Verification:** Run the simulator with motion disabled. Verify that:
  1. Messages propagate correctly through the chain.
  2. Pressure gradients settle (Source has high pressure, Sink has low pressure, Relay is in between).
  3. Edge conductivities converge to positive values.

---

## Phase 3: Motion Mechanism (Soap Bubble Optimization & Heuristics)
Develop the spatial movement controller based on attraction, repulsion, and angular alignment.

### Task 3.1: Calculate Local Penalties (Soap Bubble Optimization)
* **Mini-Task 3.1.1:** Implement the repulsion force calculation:
  $$Fr_{ij} = \frac{1}{L_{ij}^2}$$
* **Mini-Task 3.1.2:** Implement the angular penalty function:
  $$P_{\theta_{jk}^i} = \cos \theta_{jk}^i + 1$$
* **Mini-Task 3.1.3:** Combine these into the total penalty function $Z_i$:
  $$Z_i = w_p \sum Fr + (1 - w_p) \sum P_{\theta}$$

### Task 3.2: Implement Position Evaluation
* **Mini-Task 3.2.1:** Implement the available location grid $L(i)$ representing the discretized grid cells of size $2d_r$ within radius $d_s$ partitioned into $2n$ sectors, exactly as described in Section 3.4 of the paper.
* **Mini-Task 3.2.2:** Filter out candidate positions in $L(i)$ that are blocked by obstacles (read from rangefinders) or other robots.
* **Mini-Task 3.2.3:** For each available candidate position, evaluate the penalty $Z_i$ and find the location $l_i'$ that maximizes the gain $\Delta Z_{ii'} = Z_i - Z_i'$.

### Task 3.3: Implement Foraging Exploration Heuristics
* **Mini-Task 3.3.1:** Implement **Random-Walk**: Pick randomly from forward, forward-left, and forward-right.
* **Mini-Task 3.3.2:** Implement **Wobble**: Move orthogonally to neighbors to test alternative paths.
* **Mini-Task 3.3.3:** Implement **Spread**: Move in the direction of cumulative repulsive vectors.
* **Mini-Task 3.3.4:** Implement the `should-explore?()` decision rule.

---

## Phase 4: Swarm Network Optimization (SNO) Integration
Tie the adaptation (Di-PL) and motion phases together inside `ControlStep()`.

### Task 4.1: Integrate Controller Logic Loop
* **Mini-Task 4.1.1:** Implement the two-phase execution cycle (Algorithm 2) in the controller `ControlStep()`:
  1. **Adaptation Phase:** Process received messages, run Di-PL updates, update state, and broadcast outgoing flux messages.
  2. **Motion Phase:** Assess exploration vs. exploitation. If exploiting, execute Soap Bubble optimization and set differential drive velocities.
* **Mini-Task 4.1.2:** Implement the food decay factor $\gamma_Q$ to handle scenarios without nests.
* **Mini-Task 4.1.3:** Test compilation and resolve any memory or symbol linkage issues.

---

## Phase 5: Loop Functions & Metrics Tracking
Use ARGoS loop functions to define experiment logic and output evaluation metrics.

### Task 5.1: Implement Source & Sink Logic
* **Mini-Task 5.1.1:** Create a new custom loop function class ([phybot_loop_functions.cpp](loop_functions/phybot_loop_functions.cpp)).
* **Mini-Task 5.1.2:** Configure the loop function to designate specific physical locations or entities as **Sources** (injecting flow) and **Sinks** (consuming flow) and set the roles of robots in contact with them.
* **Mini-Task 5.1.3:** Implement dynamic targets (moving light entities) and reassign robot roles dynamically.

### Task 5.2: Track Evaluation Metrics
Save metrics to a log file at the end of each simulation tick.
* **Mini-Task 5.2.1:** Track **Terminal Connectivity**: Number of connected network components.
* **Mini-Task 5.2.2:** Track **Sum of Minimal Inter-Agent Distances** to monitor swarm density and convergence.
* **Mini-Task 5.2.3:** Calculate the **Chamfer Distance** to the optimal Steiner Tree configuration. **Dependency Note:** Load the pre-computed GeoSteiner benchmark coordinates from a text file or hardcode them directly inside the loop functions.

---

## Phase 6: Validation & Experimentation
Run tests mimicking the article's experiments to verify that the implementation works correctly.

### Task 6.1: Minimal Swarm Sanity Check
* **Task:** Run a small simulation with 3-4 mobile robots.
* **Verification:** Verify that the Braitenberg/SBO motion reacts correctly to repulsion and angle changes, and the robots form a stable line connecting source and sink.

### Task 6.2: Run Ring & Maze Experiments
* **Mini-Task 6.2.1:** Setup a ring-shaped arena layout in a configuration file `experiments/phybot_ring.argos`. Verify that the robot swarms prune the longer route and converge to the shorter one.
* **Mini-Task 6.2.2:** Setup the maze environment in `experiments/phybot_maze.argos`. Test if the robots successfully find the path and form a straight chain connecting the nest and food.
* **Mini-Task 6.2.3:** Validate that the parameters ($w_p = 0.375$ for mazes) match the article's findings.

### Task 6.3: Run Polygon & Large Steiner Configurations
* **Mini-Task 6.3.1:** Place food sources in polygonal shapes (triangle, square, pentagon) and verify if the swarm forms Steiner-like junctions.
* **Mini-Task 6.3.2:** Test adaptability with expendable food sources (sources that disappear when depleted and respawn elsewhere). Check if the swarm dynamically disperses and forms new chains.
* **Mini-Task 6.3.3:** Run large-scale simulations (e.g., 15 terminals and 150 Phybots) and plot metrics to compare with the paper's benchmarks.
