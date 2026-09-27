# VibroApp

![VibroApp](imager.png)

Real-time vibration monitor for sensor data with live time-domain and FFT plots for X, Y, and Z axes. AI generated code.

## Features
- Reads vibration data from a connected sensor over serial
- Counts data packets and warns the user in the event of packet loss.
- Shows live waveform and FFT charts for all three axes
- Displays RMS vibration and dominant FFT peaks
- Auto reconnect to sensor.
- Provides basic ESC power control for FC-connected motors
- Saves snapshot of data to files or/and screenshot snapshot of graphs

## Requirements
- Python 3.10+
- PyQt6
- pyqtgraph
- numpy
- psutil
- pyserial

Install dependencies:

```bash
pip install PyQt6 pyqtgraph numpy psutil pyserial
```

## Run

```bash
python vibro.py
```

If the sensor port is not detected automatically, the app will ask you to select the serial port manually.

### Check for updates

The startup **Select mode** window has a **Check for updates** button beneath the binary-file input. It checks the latest stable GitHub release without blocking the UI. If a newer version is found, you can approve a fast-forward-only `git pull`; after a successful update, close the dialog and restart VibroApp.

Automatic updates require all of the following:
- VibroApp was installed by cloning `https://github.com/sentoxo/vibro.git` (not by downloading a ZIP).
- Git is installed and available on `PATH`.
- The clone is on the `main` branch, its `origin` points to the project repository, and it has no local or untracked changes.
- The app can reach GitHub. No GitHub login is needed for the public repository.

If the clone has local changes, commit, stash, or back them up before updating. Updates use `git pull --ff-only origin main`; they will not overwrite changes or create a merge commit. On failure, the app reports the Git error and leaves the dialog open.

#### Publishing an update

The installed version is `0.1.0` (beta). To publish a later stable version:
1. Update `APP_VERSION` in `config.py` and merge the tested release commit into `main`.
2. Create a GitHub Release with a matching stable tag such as `v0.1.1`, from a commit already on `main`.
3. Ensure `main` contains the complete release before publishing. The app checks the latest stable release tag, then pulls `main`, so the release commit must be reachable from `main`.

The updater checks GitHub's latest stable release; prereleases are not offered. Until the first stable GitHub Release exists, the update check will report that no stable release was found.

### File mode

Open a binary burst file (DVB1) instead of asking for a mode:

```bash
python vibro.py --file path/to/data.bin
```

### Debug mode

Validate a burst file and exit without opening the GUI:

```bash
python vibro.py --debug --file path/to/data.bin
```

## Project Structure

The code is split into small modules by concern. `vibro.py` is the only entry point — run it with `python vibro.py`.

| Module | Responsibility |
| --- | --- |
| `config.py` | Constants and the `SEQ_LINE` regex |
| `utils.py` | Gravity removal, DVB1 burst parsing, port discovery |
| `dialogs.py` | Startup mode dialog and port-selection dialog |
| `serial_io.py` | `SerialWorker` thread: reads ESC32 burst lines |
| `esc_telemetry.py` | `FCTelemetryReader` thread: Betaflight motor-RPM telemetry |
| `fft_worker.py` | `FFTWorker` thread: amplitude spectra for submitted chunks |
| `updater.py` | GitHub release checks and guarded Git fast-forward updates |
| `app.py` | `RealtimeVibeApp` orchestrator (UI + plotting + threads) |
| `vibro.py` | Entry point: argument parsing and app startup |

## Notes
- The app expects ESP32 data in the format used by this project.
- FFT processing is handled in a worker thread to reduce UI lag.
- Each FFT axis scales independently from its current spectrum, with a minimum range of 0–2 m/s².
- ESC control is intended for testing and should be used carefully.
