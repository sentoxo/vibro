# FFT worker thread: computes amplitude spectra for submitted signal chunks.

import threading
from collections import deque

import numpy as np
from PyQt6.QtCore import QThread, pyqtSignal

import config


class FFTWorker(QThread):
    result_ready = pyqtSignal(int, object, object)

    def __init__(self):
        super().__init__()
        self.queue = deque()
        self.lock = threading.Lock()

    def submit(self, axis_index, signal_arr, sample_rate=config.FS, full=False):
        with self.lock:
            self.queue.append((axis_index, signal_arr, sample_rate, full))

    def run(self):
        while not self.isInterruptionRequested():
            item = None
            with self.lock:
                if self.queue:
                    item = self.queue.popleft()

            if item is None:
                self.msleep(10)
                continue

            axis_index, signal_arr, sample_rate, full = item
            freqs, amps = self.compute_fft(signal_arr, sample_rate, full)
            self.result_ready.emit(axis_index, freqs, amps)

    @staticmethod
    def compute_fft(signal_arr, sample_rate=config.FS, full=False):
        arr = np.asarray(signal_arr, dtype=np.float64)
        n = len(arr)
        if n < 64:
            return np.array([]), np.array([])

        if not full:
            # Live mode: cap FFT length for speed
            n = min(n, config.MAX_FFT_POINTS)
            if len(arr) > n:
                arr = arr[-n:]

        sig = arr - np.mean(arr)
        window = np.hanning(n)
        scale = np.sum(window) / n

        yf = np.fft.rfft(sig * window)
        freqs = np.fft.rfftfreq(n, 1.0 / sample_rate)
        amp = (np.abs(yf) / n) / scale
        if n > 1:
            amp[1:-1] *= 2.0

        return freqs, amp
