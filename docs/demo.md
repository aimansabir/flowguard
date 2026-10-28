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
docker run --rm -it --init --name flowguard -p 127.0.0.1:8765:8765 flowguard `
    --controller maxpressure_hyst_cap --guard v1 --faults smoke --seconds 300
```

In a second PowerShell window:

```powershell
Start-Process "http://127.0.0.1:8765/"
```

The simulation waits until the page connects, then runs. On the page:
Pause/Play (or the space bar), the speed selector (simulated seconds per
second), the timeline slider to scrub back and "Live" to return.

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

## What the page shows

- **Corridor** (Karachi left-hand traffic): vehicles by class (red outline =
  stopped), stop-line bars in the envelope's signal colours, a light dot
  where SUMO shows a give-way green (the right turn across oncoming traffic),
  camera zones coloured by the Guard's verdict (green trusted, amber suspect,
  red faulty), `VIRTUAL` where the controller is fed the virtual sensor,
  `fault: ...` where the experiment injects a fault, `no data` where a
  measurement did not arrive.
- **Junction cards (Module 3)**: phase and stage, time in phase against the
  minimum green, the max-pressure term of each phase, and why the phase was
  kept or switched, including envelope interventions.
- **Module 2 / Guard**: per camera the injected fault (experiment ground
  truth, never given to the core), trust state, camera queue, the queue the
  controller is given with its margin (highlighted when VIRTUAL), and the
  E1-E5 evidence (from the parity-tested Python copy; E6 and the combined
  score are internal to the core). A timeline of every camera's trust.
- **Module 1 / camera**: what the camera reported per approach this second
  (vehicles in the zone, stopped, arrived, left by movement, image health).
  In SUMO runs the camera is simulated (`source: sim`) and reports no image
  health; video-derived measurements say `source: camera`.
- **Queues**: camera, the controller's estimate and SUMO's true queue on the
  whole approach (reference only), and the phase timeline and decision log.
