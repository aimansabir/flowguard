# Running the FlowGuard demo on Windows

The whole system (SUMO, the observer, the fault injector, the Rust core with
Guard v1 and the controllers, the safety envelope and the live dashboard)
runs in a Linux container. Windows only runs Docker Desktop and the browser,
so nothing native is compiled on the laptop (Smart App Control stays on).
The dashboard is published on 127.0.0.1 only.

## Once: build the image

PowerShell, in the repository folder:

```powershell
Start-Process "$env:LOCALAPPDATA\Programs\DockerDesktop\Docker Desktop.exe"
while (-not (docker info 2>$null)) { Start-Sleep 3 }          # wait for the engine
docker build -t flowguard -f docker/Dockerfile .
```

The first build downloads Rust crates, Python and SUMO (several minutes);
later builds reuse them.

## Run a scenario and watch it

```powershell
# smoke: every fault kind within 5 minutes, Guard v1, Khizer's max-pressure with hysteresis and spillback gate
docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 `
    -v "${PWD}\results:/app/results" flowguard `
    --controller maxpressure_hyst_cap --guard v1 --faults smoke --seconds 300
```

The `-v` line is optional: it lets the Research view read the runs in
`results\` (read-only) and keeps this run's logs there. Without it the run
still works and Research shows an empty state.

In a second PowerShell window:

```powershell
Start-Process "http://127.0.0.1:8765/"
```

The simulation waits until the page connects, then runs. In the header:
play/pause (or the space bar), the speed selector (simulated seconds per
second; at "max" a run goes as fast as SUMO and the core allow), the clock,
and under it the timeline slider.

Rewind and replay: drag the timeline slider back to any second (or click the
Overview timeline, a Guard event or a decision-log entry) and every screen
shows that moment. Press play there to watch it again from that point at the
selected speed; the live simulation keeps running meanwhile, and "Back to
live" returns to it. Once a run has finished, play replays it from the start.
The replay uses the seconds the page has already received; it does not
re-run SUMO. To run the same simulation again from scratch, start the same
`docker run` command again: the same seed gives the same traffic.

Other runs (stop the previous one first):

```powershell
# one hour, every fault kind at once (the guard spec's example)
docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 flowguard `
    --controller maxpressure_hyst_cap --guard v1 --faults mixed --seconds 3600 --speed 16

# a single frozen camera (J1_J2, 900-1500 s)
docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 flowguard `
    --controller maxpressure_hyst_cap --guard v1 --faults freeze --seconds 1800 --speed 8

# the same faults without the Guard: the controller trusts every camera
docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 flowguard `
    --controller maxpressure_hyst_cap --guard passthrough --faults mixed --seconds 3600 --speed 16

# other controllers: fixed, fixed_webster, fixed_webster_wave, maxpressure, maxpressure_hyst, maxpressure_hyst_cap
docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 flowguard `
    --controller fixed_webster --guard v1 --faults smoke --seconds 300
```

Fault scenarios are the files in `config/faults/`: `smoke`, `mixed`,
`freeze`, `blackout`, `undercount`, `dropout`, `displaced`.

## Stop

`Ctrl+C` in the window running the container, or from any window:

```powershell
docker stop flowguard
```

## Keep the run's logs

Add `-v "${PWD}\results:/app/results"` to `docker run` to get the run
directory (measurements, states, commands, faults, summary) in `results\`,
then score the Guard with

```powershell
docker run --rm -v "${PWD}\results:/app/results" --entrypoint python flowguard `
    -m flowguard_sim.guard_eval /app/results/<run directory>
```

## Replay a recorded run

```powershell
docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 `
    -v "${PWD}\results:/app/results" flowguard `
    --controller maxpressure_hyst_cap --guard v1 --speed 32 `
    --replay results/<folder>/<run directory>
```

The recorded measurements are fed to the core again (open loop). Replays
show the run's injected faults; vehicle positions are not recorded, so the
map draws signals and camera states only.

## What the page shows

Five views share one stream: switching between them does not restart the
simulation, and the selected camera and junction carry over.

- **Overview**: the corridor from the SUMO network (Karachi left-hand
  traffic), vehicles by class (red tail = stopped), stop-line bars and
  signal heads in SUMO's own signal colours (one approach per junction is
  green at a time; all its movements are protected, D32), camera zones coloured by
  the Guard (blue trusted, amber suspect, red faulty) with `VIRTUAL` where
  the controller is fed the virtual sensor and `NO DATA` where a measurement
  did not arrive. Click a camera to open it in Guard, a junction badge to
  open it in Controller. Below: one card per module, the current event, and
  the run's fault and recovery timeline (click to jump).
- **Vision (Module 1)**: the selected camera's measurement this second
  (vehicles per class in the zone, stopped, arrivals, departures by
  movement, queue) and its image health. In SUMO runs the camera is
  simulated and labelled so; there is no video. The view is rendered from
  SUMO (camera perspective or top-down) with boxes on the vehicles SUMO
  places in the zone, and the counts the camera reported beside them. Video, YOLO boxes and track IDs appear only when the
  video pipeline supplies them.
- **Guard (Module 2)**: trust per camera; for the selected one the camera
  value against the value the controller receives (with its margin), SUMO's
  queue for reference, the injected fault (ground truth, never given to the
  core), a queue chart with the fault interval and trust transitions, a
  timeline, recovery status, and the E1-E5 evidence from the parity-tested
  Python copy (E6 and the combined score live only in the core).
- **Controller (Module 3)**: per junction the green approach, the stage,
  time in phase against the minimum green, the pressure of each approach
  (one phase per approach, D32), requested against approved approach; for
  the selected junction the reason for keeping or switching
  (hysteresis, spillback gate, envelope), the pressure calculation, a phase
  timeline and the decision log. Parameters are under a fold.
- **Research**: the experiment sets under `results\`: completed runs,
  traffic performance with and without the Guard by scenario, and the
  Guard's detection rate, delay, false alarms, recovery and virtual-sensor
  coverage (scored with `guard_eval` when the view opens; a one-hour set of
  48 runs takes a few minutes). Labelled preliminary.
