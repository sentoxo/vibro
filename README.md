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
| `app.py` | `RealtimeVibeApp` orchestrator (UI + plotting + threads) |
| `vibro.py` | Entry point: argument parsing and app startup |

## Notes
- The app expects ESP32 data in the format used by this project.
- FFT processing is handled in a worker thread to reduce UI lag.
- Each FFT axis scales independently from its current spectrum, with a minimum range of 0–2 m/s².
- ESC control is intended for testing and should be used carefully.
