#!/usr/bin/env python3
"""Downsample a PCD prior map while preserving its point fields.

The default target is a lightweight localization prior map for SuperOdom:
keep about 20% of the original points with a fixed random seed.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = REPO_ROOT.parent / "data" / "active_maps" / "all_field_origin_down_0p5m.pcd"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Reduce a PCD prior map to an approximate point ratio."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        default=DEFAULT_INPUT,
        help=f"Input PCD path. Default: {DEFAULT_INPUT}",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output PCD path. Default: <input_stem>_<ratio>pct.pcd",
    )
    parser.add_argument(
        "--keep-ratio",
        type=float,
        default=0.2,
        help="Approximate fraction of points to keep. Default: 0.2",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducible sampling. Default: 42",
    )
    return parser.parse_args()


def read_header(path: Path) -> Tuple[List[bytes], int, Dict[str, str], str]:
    header_lines: List[bytes] = []
    metadata: Dict[str, str] = {}

    with path.open("rb") as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"{path} ended before DATA line")
            header_lines.append(line)
            stripped = line.decode("utf-8", errors="strict").strip()
            if stripped and not stripped.startswith("#"):
                key, _, value = stripped.partition(" ")
                metadata[key.upper()] = value.strip()
                if key.upper() == "DATA":
                    return header_lines, f.tell(), metadata, value.strip().lower()


def output_path_for(input_path: Path, output_path: Path | None, keep_ratio: float) -> Path:
    if output_path is not None:
        return output_path
    percent = int(round(keep_ratio * 100.0))
    return input_path.with_name(f"{input_path.stem}_{percent}pct{input_path.suffix}")


def choose_indices(point_count: int, keep_ratio: float, seed: int) -> np.ndarray:
    if not 0.0 < keep_ratio <= 1.0:
        raise ValueError("--keep-ratio must be in the range (0, 1]")
    keep_count = max(1, int(round(point_count * keep_ratio)))
    if keep_count >= point_count:
        return np.arange(point_count, dtype=np.int64)
    rng = np.random.default_rng(seed)
    indices = rng.choice(point_count, size=keep_count, replace=False)
    indices.sort()
    return indices


def pcd_dtype(metadata: Dict[str, str]) -> np.dtype:
    fields = metadata["FIELDS"].split()
    sizes = [int(value) for value in metadata["SIZE"].split()]
    types = metadata["TYPE"].split()
    counts = [int(value) for value in metadata.get("COUNT", " ".join(["1"] * len(fields))).split()]

    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("FIELDS, SIZE, TYPE, and COUNT lengths do not match")

    dtype_fields = []
    for name, size, value_type, count in zip(fields, sizes, types, counts):
        key = (value_type.upper(), size)
        if key == ("F", 4):
            dtype = "<f4"
        elif key == ("F", 8):
            dtype = "<f8"
        elif key == ("I", 1):
            dtype = "i1"
        elif key == ("I", 2):
            dtype = "<i2"
        elif key == ("I", 4):
            dtype = "<i4"
        elif key == ("I", 8):
            dtype = "<i8"
        elif key == ("U", 1):
            dtype = "u1"
        elif key == ("U", 2):
            dtype = "<u2"
        elif key == ("U", 4):
            dtype = "<u4"
        elif key == ("U", 8):
            dtype = "<u8"
        else:
            raise ValueError(f"Unsupported PCD field type/size: TYPE={value_type} SIZE={size}")

        if count == 1:
            dtype_fields.append((name, dtype))
        else:
            dtype_fields.append((name, dtype, (count,)))

    return np.dtype(dtype_fields)


def updated_header_lines(header_lines: Iterable[bytes], point_count: int) -> List[bytes]:
    updated: List[bytes] = []
    for raw_line in header_lines:
        line = raw_line.decode("utf-8")
        key = line.strip().partition(" ")[0].upper()
        if key == "WIDTH":
            updated.append(f"WIDTH {point_count}\n".encode("utf-8"))
        elif key == "HEIGHT":
            updated.append(b"HEIGHT 1\n")
        elif key == "POINTS":
            updated.append(f"POINTS {point_count}\n".encode("utf-8"))
        else:
            updated.append(raw_line)
    return updated


def downsample_ascii(
    input_path: Path,
    output_path: Path,
    header_lines: List[bytes],
    data_offset: int,
    point_count: int,
    indices: np.ndarray,
) -> None:
    wanted = set(int(index) for index in indices)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with input_path.open("rb") as src, output_path.open("wb") as dst:
        src.seek(data_offset)
        dst.writelines(updated_header_lines(header_lines, len(indices)))
        for point_index, line in enumerate(src):
            if point_index in wanted:
                dst.write(line)
            if point_index + 1 >= point_count:
                break


def downsample_binary(
    input_path: Path,
    output_path: Path,
    header_lines: List[bytes],
    data_offset: int,
    metadata: Dict[str, str],
    point_count: int,
    indices: np.ndarray,
) -> None:
    dtype = pcd_dtype(metadata)
    with input_path.open("rb") as src:
        src.seek(data_offset)
        points = np.fromfile(src, dtype=dtype, count=point_count)
    if len(points) != point_count:
        raise ValueError(f"Expected {point_count} points, read {len(points)}")

    sampled = points[indices]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as dst:
        dst.writelines(updated_header_lines(header_lines, len(sampled)))
        sampled.tofile(dst)


def main() -> int:
    args = parse_args()
    raw_input_path = args.input.expanduser()
    input_path = raw_input_path.resolve()
    if not input_path.exists():
        raise FileNotFoundError(input_path)

    output_path = output_path_for(raw_input_path, args.output, args.keep_ratio).expanduser().resolve()
    header_lines, data_offset, metadata, data_type = read_header(input_path)
    point_count = int(metadata["POINTS"])
    indices = choose_indices(point_count, args.keep_ratio, args.seed)

    if data_type == "ascii":
        downsample_ascii(input_path, output_path, header_lines, data_offset, point_count, indices)
    elif data_type == "binary":
        downsample_binary(input_path, output_path, header_lines, data_offset, metadata, point_count, indices)
    else:
        raise ValueError(f"Unsupported PCD DATA type: {data_type}")

    actual_ratio = len(indices) / point_count
    print(f"input:  {input_path}")
    print(f"output: {output_path}")
    print(f"points: {point_count} -> {len(indices)} ({actual_ratio:.2%})")
    print(f"data:   {data_type}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
