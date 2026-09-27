# Module-level configuration constants and the SEQ_LINE regex.
# No behavior lives here — only values and one compiled pattern.

import re

# === CONFIGURATION ===
BAUD_RATE = 921600
FC_BAUD_RATE = 115200
FS = 800.0
WINDOW_SECONDS = 2.0
BUFFER_SIZE = int(FS * WINDOW_SECONDS)
LSB_TO_MS2 = 0.0039 * 9.80665
FFT_Y_MAX = 2.0
FFT_Y_SCALE_STEPS = (1, 2, 3, 4, 5, 6, 8, 10)
TIME_Y_MIN_RANGE = 2.0
DEFAULT_FFT_REFRESH_HZ = 10.0
FFT_UPDATE_INTERVAL = 1.0 / DEFAULT_FFT_REFRESH_HZ
MAX_FFT_POINTS = 512
ESP_RECONNECT_INTERVAL_MS = 1000   # wait between auto-reconnect attempts

SEQ_LINE = re.compile(r"^S(\d),(\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
PREFERRED_PORT_DESCRIPTION = "USB_SERIAL CH340"

# === DVB1 BINARY BURST FORMAT ===
DVB1_MAGIC = b"DVB1"
DVB1_HEADER_SIZE = 4 + 2 + 2 + 4 + 4 + 8 + 8  # magic|version|axes|rate|count|ts|seq
DVB1_SAMPLE_RATE = 3200.0                      # Hz
DVB1_SAMPLE_COUNT = 6400                       # samples per burst
DVB1_LSB_TO_MS2 = 0.0039 * 9.80665              # ADXL345 8g mode: 3.9 mg/LSB
FFT_MIN_HZ = 4.0                               # FFT display range
FFT_MAX_FREQ_CHOICES = [200, 400, 600, 800, 1000, 1200, 1600]  # Hz, FFT max-frequency options
DEFAULT_FFT_MAX_FREQ = 800.0                     # Hz, default FFT max frequency
IMU_HZ_CHOICES = [800, 1000, 1600, 2000, 3200, 4000]  # Hz, IMU sample-rate options
DEFAULT_IMU_HZ = 2000.0                          # Hz, default IMU sample rate
