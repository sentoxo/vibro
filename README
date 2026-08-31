# VibroApp

![VibroApp](imager.png)

Real-time vibration monitor for ESP32 sensor data with live time-domain and FFT plots for X, Y, and Z axes. AI generated code.

## Features
- Reads vibration data from a connected ESP32 over serial
- Shows live waveform and FFT charts for all three axes
- Displays average vibration and dominant FFT peaks
- Supports calibration and PNG snapshot export
- Provides basic ESC power control for FC-connected motors

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

If the ESP32 port is not detected automatically, the app will ask you to select the serial port manually.

## Notes
- The app expects ESP32 data in the format used by this project.
- FFT processing is handled in a worker thread to reduce UI lag.
- ESC control is intended for testing and should be used carefully.
