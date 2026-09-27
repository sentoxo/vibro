# Pure helper functions: gravity removal, DVB1 burst parsing, port discovery.

import struct
import time

import numpy as np
from serial.tools import list_ports

import config


def remove_gravity_component(axis_data):
    # Remove the constant (DC / gravity) component from each axis.
    return [axis - np.mean(axis) for axis in axis_data]


def parse_dvb1_file(path):
    # Read whole file and split it into bursts (DVB1 format)
    with open(path, "rb") as f:
        data = f.read()

    bursts = []
    offset = 0
    while offset + config.DVB1_HEADER_SIZE <= len(data):
        if data[offset:offset + 4] != config.DVB1_MAGIC:
            # Skip garbage byte, search for next magic
            next_magic = data.find(config.DVB1_MAGIC, offset + 1)
            if next_magic < 0:
                break
            offset = next_magic
            continue

        version, axes, sample_rate, sample_count = struct.unpack_from(
            "<HHII", data, offset + 4
        )
        timestamp_ns, sequence = struct.unpack_from("<QQ", data, offset + 16)
        samples_bytes = sample_count * axes * 2
        if offset + config.DVB1_HEADER_SIZE + samples_bytes > len(data):
            break  # truncated burst

        raw = np.frombuffer(
            data, dtype="<i2",
            count=sample_count * axes,
            offset=offset + config.DVB1_HEADER_SIZE,
        ).reshape(sample_count, axes)

        bursts.append({
            "version": version,
            "axes": axes,
            "sample_rate": float(sample_rate),
            "sample_count": sample_count,
            "timestamp_ns": timestamp_ns,
            "sequence": sequence,
            "samples": raw[:, :3].astype(np.float64) * config.DVB1_LSB_TO_MS2,
        })
        offset += config.DVB1_HEADER_SIZE + samples_bytes

    return bursts


def write_dvb1_file(path, samples, sample_rate, timestamp_ns=None, sequence=0):
    # Store physical acceleration values using the same scale the parser applies.
    samples = np.asarray(samples, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3:
        raise ValueError("DVB1 snapshots must contain exactly three axes")
    if not np.all(np.isfinite(samples)):
        raise ValueError("DVB1 snapshot samples must be finite")

    sample_rate = int(round(sample_rate))
    if sample_rate <= 0:
        raise ValueError("Sample rate must be positive")

    raw_samples = np.rint(samples / config.DVB1_LSB_TO_MS2)
    raw_samples = np.clip(raw_samples, -32768, 32767).astype("<i2")
    timestamp_ns = time.time_ns() if timestamp_ns is None else int(timestamp_ns)
    header = (
        config.DVB1_MAGIC
        + struct.pack(
            "<HHIIQQ",
            1,
            3,
            sample_rate,
            len(raw_samples),
            timestamp_ns,
            int(sequence),
        )
    )

    with open(path, "wb") as file:
        file.write(header)
        file.write(raw_samples.tobytes(order="C"))


def find_preferred_port():
    preferred = config.PREFERRED_PORT_DESCRIPTION.replace("_", " ").lower()
    for port in list_ports.comports():
        description = (port.description or "").replace("-", " ").replace("_", " ").lower()
        if preferred in description:
            return port.device
    return None
