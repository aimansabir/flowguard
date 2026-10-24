# FlowGuard — Project Design and Module Specification

## 1. Project Overview

FlowGuard is a fault-aware adaptive traffic signal control system designed for a corridor of connected intersections.

The main problem being addressed is not simply traffic congestion. The system specifically focuses on a weakness of camera-based adaptive traffic control:

> A camera can continue producing believable traffic data even when that data is wrong.

Examples include:

- a frozen video feed;
- a displaced camera;
- heavy blur;
- darkness or glare;
- partial occlusion;
- systematic vehicle undercounting;
- tracking failures;
- temporary communication loss.

A conventional adaptive controller may continue using these observations without knowing that the sensing system has become unreliable.

FlowGuard introduces a reliability layer between perception and traffic control.

The overall system is:

```text
Traffic Camera / Simulation
            │
            ▼
     MODULE 1
 Perception & Observation
            │
            ▼
       Observation
            │
            ▼
     MODULE 2
 Fault Detection & Guard
            │
            ▼
    RatedObservation
            │
            ▼
     MODULE 3
 Traffic State & Control
            │
            ▼
    Requested Signal Phase
            │
            ▼
     MODULE 4
 Safety, Execution &
 Evaluation Infrastructure
            │
            ▼
       Traffic Signal
```

The modules are deliberately separated so that each part can be developed, tested and replaced without rewriting the complete system.

---

## 2. Module 1 — Perception and Observation

### 2.1 Purpose

This module converts camera footage into a structured description of the traffic state.

Its responsibility is not to decide traffic signals.

Its responsibility is only to answer:

> What does the camera currently observe?

The output of this module is an `Observation`.

---

### 2.2 Input

In the final system:

```text
CCTV / recorded road footage
```

During development:

```text
SUMO simulator
```

SUMO currently provides ground-truth-derived observations for development and evaluation.

---

### 2.3 Computer-Vision Pipeline

The intended camera pipeline is:

```text
Video frame
     │
     ▼
Image quality analysis
     │
     ▼
Vehicle detector
     │
     ▼
Vehicle tracker
     │
     ▼
Traffic measurements
     │
     ▼
Observation
```

Possible implementation tools include:

```text
YOLO
OpenCV
ByteTrack or equivalent tracker
```

---

### 2.4 Vehicle Classes

The system currently supports six traffic classes:

```text
car
motorcycle
rickshaw
qingqi
bus
truck
```

These classes reflect the heterogeneous traffic expected on the target corridor.

---

### 2.5 Why Vehicle Count Alone is Not Enough

Ten motorcycles and ten buses should not be treated as equivalent congestion.

Therefore FlowGuard also represents queues using Passenger Car Units.

Current development weights are:

```text
car          1.0
motorcycle   0.5
rickshaw     1.0
qingqi       1.0
bus          3.0
truck        3.0
```

Example:

```text
4 cars
4 motorcycles
1 bus
```

gives:

```text
4 × 1.0
+
4 × 0.5
+
1 × 3.0

= 9 PCU
```

These values are placeholders and will later be calibrated or validated for the chosen corridor.

---

## 3. Observation Format

The perception module produces one structured observation for each monitored approach.

Conceptually:

```text
Observation
│
├── time
├── junction
├── approach
├── vehicle counts
├── queue PCU
├── vehicles leaving toward neighbours
├── detector confidence
├── tracking quality
├── image health
├── uncertainty
└── source
```

For example:

```text
Junction: J1
Approach: West → J1

Cars: 7
Motorcycles: 12
Rickshaws: 3
Bus: 1

Queue: 14.5 PCU

Vehicles leaving toward J2: 5

Detector confidence: 0.82
Image health: normal
```

The exact format is defined in:

```text
schemas/messages.py
```

and documented in:

```text
docs/schemas.md
```

---

## 4. Handling Poor Camera Quality

Poor image quality does not automatically mean that a camera has failed.

Instead, image quality becomes one source of evidence for the Guard.

The perception module will generate image-health indicators such as:

```text
frozen
blur
brightness
camera displacement
```

Other signals may include:

```text
detector confidence
track-break frequency
detection stability
```

---

### 4.1 Blurry Image

A blurred image can cause vehicle detections to become unreliable.

The system should not immediately declare:

```text
BLUR = CAMERA FAULTY
```

because temporary blur can occur naturally.

Instead:

```text
blur increases
        │
        ▼
detector confidence checked
        │
        ▼
tracking stability checked
        │
        ▼
traffic consistency checked
```

A short blur may therefore produce:

```text
TRUSTED
or
SUSPECT
```

depending on the other evidence.

Persistent severe blur combined with poor detections may eventually produce:

```text
FAULTY
```

---

### 4.2 Darkness

Night conditions can reduce detection performance.

The system can examine:

```text
frame brightness
detector confidence
tracking breaks
vehicle-count consistency
```

Darkness alone is not considered proof of failure.

The decision is based on whether the resulting traffic measurements remain credible.

---

### 4.3 Glare

Strong headlights or sunlight may temporarily reduce visibility.

The system can detect abnormal brightness or saturation.

Again, the response is evidence-based rather than immediately declaring the camera unusable.

---

### 4.4 Frozen Camera

A frozen feed is a stronger failure case.

Possible detection evidence includes:

```text
near-identical consecutive frames
+
unchanged objects/tracks
+
unchanged traffic values
+
real traffic expected to be moving
```

The Guard can also compare the camera reading with the signal command.

Example:

```text
Signal has been green for 20 seconds.

Vehicles should have moved.

Camera reports the exact same queue continuously.
```

This is evidence that the feed may be frozen.

---

### 4.5 Camera Displacement

A camera may physically move because of:

```text
wind
maintenance
impact
mount movement
```

The image may still look perfectly clear but the monitoring region may now be wrong.

Possible detection methods include:

```text
reference-image comparison
feature matching
homography / scene alignment
ROI displacement
```

A large persistent scene shift can cause the camera to be marked suspicious or faulty.

---

## 5. Partial Occlusion

A bus, truck, tree or other object may temporarily block part of the camera view.

This should not necessarily cause immediate fallback.

A likely strategy is:

```text
temporary occlusion
        ↓
higher uncertainty
        ↓
SUSPECT
```

If the camera recovers quickly:

```text
SUSPECT → TRUSTED
```

If the problem persists and measurements become inconsistent:

```text
SUSPECT → FAULTY
```

---

## 6. Output of Module 1

The key principle is:

```text
Camera / SUMO
      ↓
Observation
```

Every later component operates on this standard structure.

The controller never reads raw video.

The Guard never needs to understand SUMO internals.

This keeps the architecture modular.

---

## 7. Module 2 — Fault Detection and Guard

### 7.1 Purpose

This is the main reliability component of FlowGuard.

The Guard asks:

> Should this observation be trusted?

It receives:

```text
Observation
```

and produces:

```text
RatedObservation
```

with one of three states:

```text
TRUSTED
SUSPECT
FAULTY
```

---

## 8. Trusted State

A camera remains `TRUSTED` when its measurements are consistent with the available evidence.

Example:

```text
Camera:
queue = 15 PCU

Upstream traffic:
consistent

Signal behaviour:
consistent

Image quality:
normal

Tracking:
stable
```

Result:

```text
TRUSTED
```

The controller can use the reported queue directly.

---

## 9. Suspect State

`SUSPECT` represents uncertainty without enough evidence to declare complete failure.

Examples:

```text
temporary blur
temporary occlusion
moderate confidence drop
short tracking instability
small traffic inconsistency
```

The system may respond by increasing the uncertainty margin.

Conceptually:

```text
camera reports 15 PCU

Guard says:

estimate = 15
margin = ±5
state = SUSPECT
```

The controller can then behave more conservatively.

---

## 10. Faulty State

A sensor becomes `FAULTY` when sufficient evidence indicates that the reported state should no longer be trusted.

Examples include:

```text
frozen feed
persistent undercount
major camera displacement
complete blackout
persistent detector collapse
```

At that point the system should avoid directly using the camera's measurement.

---

## 11. Why One Camera Cannot Validate Itself

A core design principle is:

> A sensor should not be the only source used to decide whether that same sensor is trustworthy.

For example:

```text
Camera says:
"I see only 2 vehicles."

Detector confidence says:
"I am confident."
```

That does not guarantee correctness.

The camera may simply be confidently wrong.

Therefore the Guard uses evidence that is partly independent of the camera.

---

## 12. Guard Evidence Source 1 — Image Health

The Guard can use:

```text
freeze detection
blur
brightness
glare
camera displacement
```

This is useful for visible camera failures.

However, image-health information alone is not sufficient.

A perfectly clear image can still produce incorrect traffic measurements.

---

## 13. Guard Evidence Source 2 — Detector and Tracking Behaviour

The perception system can report:

```text
mean confidence
confidence variation
track breaks
unusual detection changes
```

Example:

```text
Normal:
1–2 track breaks/minute

Current:
20 track breaks/minute
```

This may indicate that detection quality has degraded.

---

## 14. Guard Evidence Source 3 — Cross-Junction Traffic Consistency

This is one of the important network-level checks.

Consider:

```text
J1 ─────────────→ J2
```

Suppose 20 vehicles leave J1 toward J2.

After a realistic travel time, some corresponding vehicles should appear near J2.

Conceptually:

```text
Vehicles leaving J1
        │
        ▼
expected travel time
        │
        ▼
Vehicles approaching J2
```

If J1 consistently reports vehicles leaving but J2's camera keeps reporting an empty road, J2 may be suspicious.

This check is useful because it uses information from another location.

---

## 15. Travel-Time Window

Vehicle movement between junctions is not instantaneous.

Therefore FlowGuard should not perform:

```text
J1 exit now
=
J2 arrival now
```

Instead it uses a travel-time window.

For example:

```text
J1 exit at t = 100 s

expected J2 arrival:

approximately
t = 115–130 s
```

The exact window depends on:

```text
distance
speed
congestion
signal state
```

---

## 16. Guard Evidence Source 4 — Command Response

The Guard also receives information about previous signal commands.

This gives it another physical consistency check.

Example:

```text
Large queue exists.

Signal changes to green.

Vehicles should begin leaving.
```

Expected behaviour:

```text
queue decreases
```

But the more correct formulation is:

```text
expected queue change
=
arrivals
-
departures
```

because a queue can still grow during green if arrivals are very high.

FlowGuard therefore compares:

```text
expected queue evolution
```

against:

```text
camera-observed queue evolution
```

Large persistent disagreement becomes evidence of a sensing problem.

---

## 17. Systematic Undercounting

This is one of the harder failure cases.

Suppose the actual queue is:

```text
30 PCU
```

but the camera consistently reports:

```text
10 PCU
```

The video may not be frozen.

The image may not be blurry.

Detector confidence may even remain high.

Therefore this failure cannot be detected using image quality alone.

FlowGuard instead checks network evidence.

For example:

```text
many vehicles entered the road
few were observed leaving
but camera says almost no queue exists
```

The values do not satisfy expected traffic conservation.

The Guard can therefore become suspicious of the observation.

---

## 18. Fault Injection

Before testing these ideas on uncontrolled real faults, the simulator deliberately creates known sensor failures.

Examples:

```text
freeze
blackout
undercount
temporary detector collapse
```

The architecture becomes:

```text
SUMO TRUE STATE
       │
       ├──────────────→ evaluation ground truth
       │
       ▼
Fault Injector
       │
       ▼
Corrupted Observation
       │
       ▼
Guard
```

Because the true state is known, Guard performance can be measured objectively.

---

## 19. Guard Evaluation

The Guard will eventually be evaluated using measures such as:

```text
fault detection rate
false alarm rate
detection delay
recovery time
TRUSTED/SUSPECT/FAULTY confusion matrix
```

A useful Guard must balance two errors:

```text
missing real faults
```

and:

```text
incorrectly declaring good cameras faulty
```

---

## 20. Virtual Sensor / State Reconstruction

If a camera becomes faulty, the system needs an alternative estimate.

A basic virtual sensor can use:

```text
previous queue estimate
+
estimated arrivals
-
estimated departures
```

Conceptually:

```text
Q(t+1)
=
Q(t)
+
A(t)
-
D(t)
```

where:

```text
Q = estimated queue
A = estimated arrivals
D = estimated departures
```

Arrival estimates can come from:

```text
upstream exit counts
travel time
historical traffic
```

Departure estimates can use:

```text
current signal phase
estimated discharge rate
```

---

## 21. Graceful Degradation

FlowGuard is designed to degrade gradually instead of treating every fault identically.

Conceptually:

```text
TRUSTED
   ↓
normal adaptive control

SUSPECT
   ↓
higher uncertainty
more conservative decision

FAULTY
   ↓
replace camera state
with virtual estimate

multiple severe failures
   ↓
fallback strategy
```

This prevents a single camera problem from unnecessarily disabling the entire corridor.

---

## 22. Module 3 — Traffic State and Adaptive Control

### 22.1 Purpose

This module decides which compatible traffic movements should receive green.

It does not directly control physical signal transitions.

Its output is only:

```text
requested phase
```

---

## 23. Current Controllers

The repository currently contains two controllers:

```text
Fixed-time
Basic max-pressure
```

An actuated controller may also be added as another engineering baseline.

---

## 24. Fixed-Time Controller

The fixed controller follows a predefined plan regardless of traffic.

Current development timing:

```text
Main-road green: 40 s
Side-road green: 20 s
```

The safety envelope adds:

```text
yellow
+
all-red
```

during transitions.

This controller provides a simple conventional baseline.

The final fixed-time plan should be calibrated for the selected corridor before final evaluation.

---

## 25. Max-Pressure Control

Max-pressure makes decisions using both:

```text
traffic waiting upstream
```

and:

```text
congestion downstream
```

Basic idea:

```text
pressure
=
upstream queue
-
downstream queue
```

Example:

```text
Road A:

20 waiting
18 downstream

pressure = 2
```

Another road:

```text
Road B:

12 waiting
1 downstream

pressure = 11
```

Road B may be preferred because vehicles can actually move forward.

---

## 26. Why Downstream Traffic Matters

A controller that only serves the longest queue may create another problem.

Example:

```text
J1 queue is large
        ↓
controller gives green
        ↓
J1 sends vehicles toward J2
        ↓
J1→J2 road is already full
        ↓
vehicles cannot clear J1
        ↓
spillback / gridlock
```

Max-pressure attempts to avoid blindly pushing vehicles into already congested downstream links.

---

## 27. Two-Way Roads

The system treats each direction separately.

For example:

```text
J1 ─────────→ J2
    J1_J2

J1 ←───────── J2
    J2_J1
```

Congestion going toward J2 does not automatically mean congestion going toward J1.

The two directions are independently represented.

---

## 28. Signal Phases

The current development intersection has two simple compatible phases.

```text
PHASE 0

WEST ←──── JUNCTION ────→ EAST
            GREEN

NORTH / SOUTH
RED
```

Then:

```text
PHASE 1

WEST / EAST
RED

       NORTH
         ↑
         │
      JUNCTION
         │
         ↓
       SOUTH

       GREEN
```

The pressure of compatible approaches is combined when selecting a phase.

---

## 29. Capacity-Aware Max-Pressure

The current controller subtracts downstream queue.

The future version should also consider:

> How much space does the downstream road actually have?

For example:

```text
Road 1:
20 vehicles downstream
capacity = 100

Road 2:
20 vehicles downstream
capacity = 25
```

These are clearly not equally congested.

Therefore the improved controller should account for:

```text
downstream occupancy / capacity
```

rather than queue alone.

This is the planned capacity-aware extension.

---

## 30. Spillback Protection

A downstream link approaching capacity can block an upstream intersection.

The controller should therefore become increasingly reluctant to release additional vehicles toward a saturated link.

This mechanism is important for corridor-level control because the objective is not merely to reduce one queue.

The system must prevent congestion from propagating backwards through multiple junctions.

---

## 31. Short-Horizon Arrival Information

A future extension may use short-horizon upstream arrival estimates.

Example:

```text
J1 currently releases
a large vehicle group
        ↓
those vehicles will reach J2
soon
        ↓
J2 controller can prepare
```

This can help coordinate adjacent intersections without requiring a complex reinforcement-learning system.

---

## 32. Module 4 — Safety, Execution and Evaluation Infrastructure

### 32.1 Purpose

This module ensures that:

```text
controller decisions are applied safely
+
experiments are reproducible
+
performance can be measured
```

It contains:

```text
Safety Envelope
SUMO execution
logging
metrics
tests
experiment runner
configuration
```

---

## 33. Safety Envelope

The controller does not directly control the traffic signal.

Instead:

```text
Controller
    ↓
"I request phase 1"

Safety Envelope
    ↓
"Can phase 1 legally start?"

Traffic Signal
```

This prevents optimization logic from bypassing basic signal rules.

---

## 34. Current Safety Constraints

Current development settings include:

```text
minimum green = 10 s

yellow = 3 s

all-red = 2 s

maximum red = 90 s
```

These values remain configurable.

Final values should be validated against the actual corridor and applicable traffic-signal practices.

---

## 35. Legal Phase Transition

A requested change cannot occur as:

```text
GREEN A
   ↓
GREEN B
```

Instead:

```text
GREEN A
   ↓
YELLOW
   ↓
ALL RED
   ↓
GREEN B
```

The controller cannot bypass this sequence.

---

## 36. Minimum Green

Suppose the controller changes its mind one second after a green starts.

The envelope rejects the immediate switch.

Example:

```text
t = 0
Phase 0 green

t = 1
Controller requests phase 1

Envelope:
NO
minimum green not completed
```

This avoids unstable and unsafe rapid switching.

---

## 37. Maximum Red

The controller cannot ignore one phase indefinitely.

If an approach has remained red too long, the safety envelope can force a phase change.

This prevents starvation.

---

## 38. Conflicting Greens

Only compatible movements are allowed to be green simultaneously.

The repository also performs an additional conflict check before sending signal states to SUMO.

Therefore invalid green combinations should be caught before execution.

---

## 39. Why Safety is Independent

This separation is deliberate.

The project may later test:

```text
max-pressure
capacity-aware max-pressure
predictive control
other adaptive strategies
```

All of them can use the same safety envelope.

Therefore changing the controller does not require rewriting signal safety.

---

## 40. SUMO as the Experimental Environment

SUMO provides a controlled digital environment where:

```text
traffic demand can be repeated
sensor faults can be injected
different controllers can be compared
ground truth remains available for evaluation
```

This is particularly important for FlowGuard.

A real camera cannot directly tell researchers the perfect true state of every vehicle.

SUMO can.

---

## 41. Ground Truth Separation

SUMO ground truth is available only to the simulation observer and evaluation logic.

The Guard and controller are intentionally prevented from directly reading simulator state.

The pipeline is:

```text
SUMO
  ↓
SimObserver
  ↓
Observation
  ↓
Guard
  ↓
Controller
```

not:

```text
SUMO
  └────────────→ Controller
```

This prevents the controller from receiving unrealistic information that would not exist in deployment.

---

## 42. Reproducible Demand

Traffic demand is generated once for a given random seed and saved.

Example:

```text
corridor_seed1.rou.xml
```

Every controller then receives:

```text
same vehicle types
same arrival times
same origins
same destinations
```

This allows fair comparison.

---

## 43. Experiment Logging

The system records several levels of information.

```text
observations.jsonl
```

contains the observation received by the Guard.

```text
rated.jsonl
```

contains the Guard output.

```text
commands.jsonl
```

contains the actual signal commands.

```text
summary.json
```

contains final experiment metrics.

This makes system behaviour traceable.

---

## 44. Current Metrics

Current metrics include:

```text
average vehicle delay
throughput
maximum queue
longest red
vehicles remaining in network
teleports
collisions
safety-envelope interventions
```

Future Guard-specific metrics will include:

```text
fault detection delay
precision
recall
false alarms
recovery time
```

---

## 45. Testing

The project includes automated tests covering several layers.

#### Message tests

Verify that:

```text
Observation
RatedObservation
Command
```

can be serialized and reconstructed correctly.

---

#### Architecture tests

Verify that Guard and controller modules do not directly import SUMO.

This protects the intended system boundary.

---

#### Safety tests

Verify:

```text
minimum green
yellow sequence
all-red sequence
maximum red
invalid phase handling
no conflicting greens
```

---

#### Smoke tests

Short complete SUMO experiments verify that:

```text
traffic runs
observations are produced
controllers operate
commands are logged
result files are generated
```

The current automated suite passes.

---

## 46. Current Implementation Status

### Implemented Foundation

```text
✓ Repository structure

✓ Observation schema

✓ RatedObservation schema

✓ Command schema

✓ Three-junction synthetic corridor

✓ Mixed vehicle classes

✓ Sublane simulation

✓ Reproducible traffic demand

✓ SUMO observer

✓ Fault-injection interface

✓ Guard interface

✓ Fixed-time controller

✓ Basic max-pressure controller

✓ Safety envelope

✓ Signal execution

✓ Metrics

✓ Logging

✓ Experiment runner

✓ Automated tests
```

---

### Under Development

```text
△ Max-pressure stabilization

△ downstream capacity handling

△ heterogeneous traffic calibration

△ fixed-time baseline tuning
```

---

### Next Implementation Stage

```text
○ real-camera perception

○ YOLO detection

○ vehicle tracking

○ image-health analysis

○ camera fault injection

○ Guard v1

○ conservation checking

○ command-response checking

○ virtual sensing

○ real corridor integration

○ multi-seed evaluation
```

---

## 47. Four Module Ownership Boundaries

The system should be developed by module rather than treating each team member as an isolated workflow.

The four modules are:

```text
MODULE 1
Perception and Observation

MODULE 2
Fault Detection and Guard

MODULE 3
Traffic State and Adaptive Control

MODULE 4
Safety, Simulation and Evaluation
```

Team members can work across modules when required.

The important rule is that module interfaces remain stable.

---

## 48. Integration Contract

The central integration path is:

```text
MODULE 1
      │
      │ Observation
      ▼
MODULE 2
      │
      │ RatedObservation
      ▼
MODULE 3
      │
      │ Requested phase
      ▼
MODULE 4
      │
      │ Safe Command
      ▼
Traffic Signal
```

This is the main architecture of FlowGuard.

---

## 49. Why This Structure is Useful

Each component can be tested independently.

For example:

```text
YOLO can improve
without changing controller
```

```text
Guard logic can improve
without changing YOLO
```

```text
controller can improve
without changing Guard
```

```text
simulation corridor can change
without redesigning the complete system
```

This reduces integration risk and makes research experiments easier to interpret.

---

## 50. Final System Behaviour

Under normal conditions:

```text
camera
  ↓
healthy observation
  ↓
TRUSTED
  ↓
adaptive control
  ↓
safe signal
```

Under degraded conditions:

```text
camera
  ↓
questionable observation
  ↓
SUSPECT
  ↓
larger uncertainty
  ↓
conservative control
```

Under failure:

```text
camera
  ↓
unreliable observation
  ↓
FAULTY
  ↓
virtual estimate / fallback
  ↓
safe continued operation
```

This is the central objective of FlowGuard:

> Adaptive traffic control should continue operating safely even when its camera sensing becomes unreliable.
