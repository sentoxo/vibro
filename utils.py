# Pure helper functions: gravity removal, DVB1 burst parsing, port discovery.

import struct

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


def find_preferred_port():
    preferred = config.PREFERRED_PORT_DESCRIPTION.replace("_", " ").lower()
    for port in list_ports.comports():
        description = (port.description or "").replace("-", " ").replace("_", " ").lower()
        if preferred in description:
            return port.device
    return None
