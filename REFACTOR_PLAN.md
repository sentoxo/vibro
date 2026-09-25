# Refactoring Plan: `vibro.py` → smaller modules

**Status:** Plan only. No code changed. `vibro.py` stays as-is until you say "go".

**Scope (confirmed):**
- Physical split only — every class and its logic stays identical; only file placement changes. No behavior change.
- Flat `.py` files in the same folder (`C:\Users\Kuba\Documents\vibro`). No Python package.
- `vibro.py` remains the runnable entry point (`python vibro.py`).
- Dead code left in place (see §6).

---

## 1. Why

`vibro.py` is 1485 lines and mixes four unrelated concerns in one file:
module constants, pure helpers, three Qt worker threads, two dialogs, and one
~1008-line orchestrator class (`RealtimeVibeApp`). Splitting by concern makes
each file independently readable and testable without altering behavior.

The dependency graph is clean and acyclic, so the split is low-risk.

## 2. Current structure (as-is)

- **Module level (1–105):** 17 constants + `SEQ_LINE` regex + 3 functions
  (`remove_gravity_component`, `parse_dvb1_file`, `find_preferred_port`).
- **Classes:** `ModeDialog`, `PortDialog`, `SerialWorker`, `FCTelemetryReader`,
  `FFTWorker`, `RealtimeVibeApp` (~1008 lines, ~40 methods).
- **Entry:** `main()` (1429–1485) with argparse + `ModeDialog` orchestration.

**Cross-reference map (what each piece needs):**
- Constants → used almost entirely by `RealtimeVibeApp`; `FS`/`MAX_FFT_POINTS`
  also used by `FFTWorker`.
- `BAUD_RATE` → `SerialWorker` creation (in app). `FC_BAUD_RATE` → app FC serial.
- `remove_gravity_component` → app (`load_burst_file`, `update_plots`).
- `parse_dvb1_file` → app (`load_burst_file`) + `main()`.
- `ModeDialog` → only `main()`; `PortDialog` → app (`reconnect_esp`).

## 3. Target layout — 8 files

| # | File | Contents | Imports |
|---|------|----------|---------|
| 1 | `config.py` | 17 constants + `SEQ_LINE` | `re` |
| 2 | `utils.py` | `remove_gravity_component`, `parse_dvb1_file`, `find_preferred_port` | `numpy`, `serial.tools.list_ports`, `config` |
| 3 | `dialogs.py` | `ModeDialog`, `PortDialog` | `PyQt6`, `config` |
| 4 | `serial_io.py` | `SerialWorker` | `config` |
| 5 | `esc_telemetry.py` | `FCTelemetryReader` | `config` |
| 6 | `fft_worker.py` | `FFTWorker` + static `compute_fft` | `config`, `numpy` |
| 7 | `app.py` | `RealtimeVibeApp` | `config`, `utils`, `dialogs`, `serial_io`, `esc_telemetry`, `fft_worker` |
| 8 | `vibro.py` | Entry: argparse + `main()` + `ModeDialog` | `app`, `dialogs` |

**Dependency direction (all point downward, no cycles):**
```
vibro.py  ->  app.py, dialogs.py
app.py    ->  config, utils, dialogs, serial_io, esc_telemetry, fft_worker
serial_io ->  config        esc_telemetry ->  config        fft_worker ->  config, utils
dialogs   ->  config        utils         ->  config
config    ->  (nothing)
```

## 4. Migration order (app stays runnable + tested after each step)

1. **`config.py`** — move the 17 constants + `SEQ_LINE`. Add explicit imports at
   the top of `vibro.py` so nothing breaks yet.
2. **`utils.py`** — move the 3 pure functions.
3. **`dialogs.py`** — move `ModeDialog`, `PortDialog`.
4. **`serial_io.py`, `esc_telemetry.py`, `fft_worker.py`** — move the 3 threads;
   each imports its constants from `config`.
5. **`app.py`** — move `RealtimeVibeApp`; add imports for the modules above and
   qualify cross-module references (e.g. `SerialWorker(...)` →
   `serial_io.SerialWorker(...)`).
6. **`vibro.py`** — strip to entry point: imports + `main()`/argparse/`ModeDialog`.

## 5. Verification (expect zero behavior change)

- `python -c "import app"` succeeds after step 5.
- `python vibro.py --help` works after step 6.
- Run the app in **demo mode** and **serial mode** (COM9): plots, FFT peaks, ESC
  ramp, and FC RPM indicators must behave identically to before.
- CPU / packet-rate unchanged (this moves code; it does not change the read logic).

## 6. Dead code (left in place per decision)

- `find_preferred_port()` — defined (line 192), never called.
- `BUFFER_SIZE` — defined, unused.

These move to their new files unchanged. Remove later in a separate cleanup pass
if desired.

## 7. Notes / open items

- `app.py` will still be ~1000 lines — that's expected under "physical split
  only"; the orchestrator class stays whole. A separation-of-concerns split
  (extracting data-model / view layers) is out of scope; request it separately if
  you want it.
- No import style changes beyond flat absolute imports (`from config import ...`).
