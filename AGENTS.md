# AGENTS.md — Instructions for AI Agents

Guidelines for working on **VibroApp**, a real-time vibration monitor for ESP32
sensor data with live time-domain and FFT plots, plus ESC/Betaflight FC control.

## Project Layout

- Flat `.py` modules in the project root. **This is NOT a Python package** —
  never use relative imports (`from . import`) or create `__init__.py`.
- Cross-module imports are absolute and reference the module name directly
  (e.g. `import config`, `import utils`, `import serial_io`).
- `vibro.py` is the **only entry point**. Run the app with `python vibro.py`.
- Keep the split by concern. Current modules:
  - `config.py` — constants and the `SEQ_LINE` regex. No logic.
  - `utils.py` — pure helpers: gravity removal, DVB1 burst parsing, port discovery.
  - `dialogs.py` — `ModeDialog` (startup) and `PortDialog` (port selection).
  - `serial_io.py` — `SerialWorker` thread (ESC32 burst lines).
  - `esc_telemetry.py` — `FCTelemetryReader` thread (Betaflight motor RPM).
  - `fft_worker.py` — `FFTWorker` thread + static `compute_fft`.
  - `app.py` — `RealtimeVibeApp` orchestrator (UI, plotting, thread wiring).
  - `vibro.py` — argument parsing and app startup.

## Core Rules

- **Write comments in code.** Explain *why*, not just *what*. Keep them concise.
- **No behavior change unless asked.** When refactoring, move code between files
  only — keep every class, method, and constant byte-for-byte identical.
- **One concern per file.** If a class or function doesn't belong, ask before
  moving it.

## Conventions

- Naming: `snake_case` for functions/variables, `PascalCase` for classes.
- Constants live in `config.py`. Import them via `config.CONSTANT`; don't
  redefine or hardcode values in logic.
- Thread classes subclass `QThread` and expose `pyqtSignal`s for results.
  Always stop threads cleanly (`stop()` / `requestInterruption()` + `wait()`).
- Keep the UI responsive: heavy work (FFT, bulk serial reads) belongs in worker
  threads, never in the main/UI thread.

## Running & Testing

- Install deps: `pip install -r requirements.txt`
  (PyQt6, pyqtgraph, numpy, psutil, pyserial).
- GUI: `python vibro.py`
- File mode: `python vibro.py --file path/to/data.bin`
- Debug (validate a burst file, no GUI): `python vibro.py --debug --file path/to/data.bin`
- GUI paths need a display; verify logic (parsing, FFT, config) with headless
  checks like `python -c "import <module>"` or the `--debug` mode.

## Key Technical Details

- **DVB1 binary burst format** (little-endian):
  `magic "DVB1" | version u16 | axes u16 | sample_rate u32 | sample_count u32 |
  timestamp_ns u64 | sequence u64 | int16 samples[count][3]`.
  Parse with `utils.parse_dvb1_file`.
- **Serial lines** match `config.SEQ_LINE`: `S<sid>,<seq>,<x>,<y>,<z>`.
- **Sample rates**: IMU live rate = `2 * fft_max_freq` (Nyquist); FFT display
  range is `FFT_MIN_HZ .. fft_max_freq`. Time-domain graphs are driven by
  `imu_hz`, independent of the FFT range.
- **Sensitivity**: `lsb_per_g` converts raw accelerometer counts to m/s².
  Default 256 (old ADXL345); 2048 for the new sensor. Applied live to the
  serial worker.
- **FFT**: `fft_worker.compute_fft` applies a Hann window and normalizes
  amplitude; `MAX_FFT_POINTS` caps length in live mode.

## Before Finishing

- Confirm all modules still import: `python -c "import config, utils, dialogs, serial_io, esc_telemetry, fft_worker, app"`.
- Confirm `python vibro.py --help` runs.
- If you changed behavior, tell the user exactly what changed and why.
- If there is new feature add it to readme.md