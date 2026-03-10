#!/usr/bin/env python3
"""MeshCam codec core module.

Implements:
- Color tile codec for 320x240 images using YCbCr 4:2:0.
- Three fixed 48-byte tile payload profiles:
    - p100: 100 data packets, Y(48x6-bit) + Cb(12x4-bit) + Cr(12x4-bit)
    - p200: 200 data packets, Y(48x6-bit) + Cb(12x4-bit) + Cr(12x4-bit)
    - p240: 240 data packets, Y(24x6-bit + 16x5-bit) + Cb(20x4-bit) + Cr(20x4-bit)
    - p300: 300 data packets, Y(32x6-bit) + Cb(16x6-bit) + Cr(16x6-bit)
- Systematic packet stream with configurable repair packets.
- Deterministic RLNC-style repair packet generation over GF(256).
- Packet loss simulation and decoder-side recovery.

The quantizer is intentionally simple and deterministic:
- p100: 4x4 luma/chroma block means
- p200: 4x2 luma and 2x4 chroma block means
- p240: 4x2 luma and 2x2 chroma block means
- p300: 4x2 luma and 2x2 chroma block means

This keeps packet independence and strict payload size while being easy to test.
"""

from __future__ import annotations

import os
import random
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

try:
    from PIL import Image  # type: ignore[reportMissingImports]

    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False


WIDTH = 320
HEIGHT = 240

# Legacy profile: 100 data packets (10x10 tiles of 32x24).
TILE_W_P100 = 32
TILE_H_P100 = 24
TILES_X_P100 = WIDTH // TILE_W_P100
TILES_Y_P100 = HEIGHT // TILE_H_P100
DATA_TILES_P100 = TILES_X_P100 * TILES_Y_P100  # 100
REPAIR_TILES_P100 = 50

Y_BLOCK_W_P100 = 4
Y_BLOCK_H_P100 = 4
C_BLOCK_W_P100 = 4
C_BLOCK_H_P100 = 4

Y_BLOCKS_PER_TILE = (TILE_W_P100 // Y_BLOCK_W_P100) * (TILE_H_P100 // Y_BLOCK_H_P100)  # 48
C_TILE_W = TILE_W_P100 // 2  # 16 (4:2:0)
C_TILE_H = TILE_H_P100 // 2  # 12
C_BLOCKS_PER_TILE = (C_TILE_W // C_BLOCK_W_P100) * (C_TILE_H // C_BLOCK_H_P100)  # 12

# Reliability-focused detail profile: 200 data packets (20x10 tiles of 16x24).
TILE_W_P200 = 16
TILE_H_P200 = 24
TILES_X_P200 = WIDTH // TILE_W_P200
TILES_Y_P200 = HEIGHT // TILE_H_P200
DATA_TILES_P200 = TILES_X_P200 * TILES_Y_P200  # 200
REPAIR_TILES_P200 = 100

Y_BLOCK_W_P200 = 4
Y_BLOCK_H_P200 = 2
Y_BLOCKS_PER_TILE_P200 = (TILE_W_P200 // Y_BLOCK_W_P200) * (TILE_H_P200 // Y_BLOCK_H_P200)  # 48

C_TILE_W_P200 = TILE_W_P200 // 2  # 8
C_TILE_H_P200 = TILE_H_P200 // 2  # 12
C_BLOCK_W_P200 = 2
C_BLOCK_H_P200 = 4
C_BLOCKS_PER_TILE_P200 = (C_TILE_W_P200 // C_BLOCK_W_P200) * (C_TILE_H_P200 // C_BLOCK_H_P200)  # 12

# High-detail profile: 300 data packets (20x15 tiles of 16x16).
TILE_W_P300 = 16
TILE_H_P300 = 16
TILES_X_P300 = WIDTH // TILE_W_P300
TILES_Y_P300 = HEIGHT // TILE_H_P300
DATA_TILES_P300 = TILES_X_P300 * TILES_Y_P300  # 300
REPAIR_TILES_P300 = 150

Y_BLOCK_W_P300 = 4
Y_BLOCK_H_P300 = 2
Y_BLOCKS_PER_TILE_P300 = (TILE_W_P300 // Y_BLOCK_W_P300) * (TILE_H_P300 // Y_BLOCK_H_P300)  # 32

C_TILE_W_P300 = TILE_W_P300 // 2  # 8
C_TILE_H_P300 = TILE_H_P300 // 2  # 8
C_BLOCK_W_P300 = 2
C_BLOCK_H_P300 = 2
C_BLOCKS_PER_TILE_P300 = (C_TILE_W_P300 // C_BLOCK_W_P300) * (C_TILE_H_P300 // C_BLOCK_H_P300)  # 16

# Balanced profile: 240 data packets (16x15 tiles of 20x16).
TILE_W_P240 = 20
TILE_H_P240 = 16
TILES_X_P240 = WIDTH // TILE_W_P240
TILES_Y_P240 = HEIGHT // TILE_H_P240
DATA_TILES_P240 = TILES_X_P240 * TILES_Y_P240  # 240
REPAIR_TILES_P240 = 120

Y_BLOCK_W_P240 = 4
Y_BLOCK_H_P240 = 2
Y_BLOCKS_PER_TILE_P240 = (TILE_W_P240 // Y_BLOCK_W_P240) * (TILE_H_P240 // Y_BLOCK_H_P240)  # 40

C_TILE_W_P240 = TILE_W_P240 // 2  # 10
C_TILE_H_P240 = TILE_H_P240 // 2  # 8
C_BLOCK_W_P240 = 2
C_BLOCK_H_P240 = 2
C_BLOCKS_PER_TILE_P240 = (C_TILE_W_P240 // C_BLOCK_W_P240) * (C_TILE_H_P240 // C_BLOCK_H_P240)  # 20

Y_HI_BITS_BLOCKS_P240 = 24
Y_LO_BITS_BLOCKS_P240 = Y_BLOCKS_PER_TILE_P240 - Y_HI_BITS_BLOCKS_P240

# Spread higher-precision Y indices across the tile to avoid concentrating detail in one region.
Y_HI_INDICES_P240 = (
    0,
    2,
    4,
    5,
    6,
    8,
    10,
    12,
    14,
    15,
    16,
    18,
    20,
    22,
    24,
    25,
    26,
    28,
    30,
    32,
    34,
    35,
    36,
    38,
)
Y_HI_INDEX_SET_P240 = set(Y_HI_INDICES_P240)
Y_LO_INDICES_P240 = tuple(i for i in range(Y_BLOCKS_PER_TILE_P240) if i not in Y_HI_INDEX_SET_P240)

PAYLOAD_SIZE = 48 + 1
PACKET_DUMP_HEADER_SIZE = 10

if len(Y_HI_INDICES_P240) != Y_HI_BITS_BLOCKS_P240:
    raise RuntimeError("p240 high-precision Y index count mismatch")
if len(Y_LO_INDICES_P240) != Y_LO_BITS_BLOCKS_P240:
    raise RuntimeError("p240 low-precision Y index count mismatch")

P240_PAYLOAD_BIT_WIDTHS = tuple(
    [6] * Y_HI_BITS_BLOCKS_P240
    + [5] * Y_LO_BITS_BLOCKS_P240
    + [4] * C_BLOCKS_PER_TILE_P240
    + [4] * C_BLOCKS_PER_TILE_P240
)
if sum(P240_PAYLOAD_BIT_WIDTHS) != (PAYLOAD_SIZE * 8):
    raise RuntimeError("p240 payload layout must be exactly 48 bytes")

COEFF_SEED = 0x5A17C3D9


@dataclass(frozen=True)
class Packet:
    frame_id: int
    symbol_id: int
    k_data: int
    n_total: int
    is_repair: bool
    payload: bytes


@dataclass(frozen=True)
class CodecProfile:
    name: str
    data_tiles: int
    repair_tiles: int 
    tile_w: int
    tile_h: int
    tiles_x: int
    tiles_y: int
    y_block_w: int
    y_block_h: int
    c_block_w: int
    c_block_h: int

PROFILE_P100 = CodecProfile("p100", DATA_TILES_P100, REPAIR_TILES_P100, TILE_W_P100, TILE_H_P100, TILES_X_P100, TILES_Y_P100, Y_BLOCK_W_P100, Y_BLOCK_H_P100, C_BLOCK_W_P100, C_BLOCK_H_P100)
PROFILE_P200 = CodecProfile("p200", DATA_TILES_P200, REPAIR_TILES_P200, TILE_W_P200, TILE_H_P200, TILES_X_P200, TILES_Y_P200, Y_BLOCK_W_P200, Y_BLOCK_H_P200, C_BLOCK_W_P200, C_BLOCK_H_P200)
PROFILE_P240 = CodecProfile("p240", DATA_TILES_P240, REPAIR_TILES_P240, TILE_W_P240, TILE_H_P240, TILES_X_P240, TILES_Y_P240, Y_BLOCK_W_P240, Y_BLOCK_H_P240, C_BLOCK_W_P240, C_BLOCK_H_P240)
PROFILE_P300 = CodecProfile("p300", DATA_TILES_P300, REPAIR_TILES_P300, TILE_W_P300, TILE_H_P300, TILES_X_P300, TILES_Y_P300, Y_BLOCK_W_P300, Y_BLOCK_H_P300, C_BLOCK_W_P300, C_BLOCK_H_P300)
CODEC_PROFILES: Dict[str, CodecProfile] = {
    PROFILE_P100.name: PROFILE_P100,
    PROFILE_P200.name: PROFILE_P200,
    PROFILE_P240.name: PROFILE_P240,
    PROFILE_P300.name: PROFILE_P300,
}


def clip_u8(v: int) -> int:
    if v < 0:
        return 0
    if v > 255:
        return 255
    return v


def quantize_u8(value: int, levels: int) -> int:
    return (value * (levels - 1) + 127) // 255


def dequantize_u8(index: int, levels: int) -> int:
    return (index * 255 + ((levels - 1) // 2)) // (levels - 1)


def build_gf_tables() -> Tuple[List[int], List[int], List[List[int]]]:
    primitive = 0x11D
    gf_exp = [0] * 512
    gf_log = [0] * 256

    x = 1
    for i in range(255):
        gf_exp[i] = x
        gf_log[x] = i
        x <<= 1
        if x & 0x100:
            x ^= primitive

    for i in range(255, 512):
        gf_exp[i] = gf_exp[i - 255]

    mul_table: List[List[int]] = [[0] * 256 for _ in range(256)]
    for a in range(256):
        if a == 0:
            continue
        for b in range(256):
            if b == 0:
                continue
            mul_table[a][b] = gf_exp[gf_log[a] + gf_log[b]]

    return gf_exp, gf_log, mul_table


GF_EXP, GF_LOG, GF_MUL = build_gf_tables()


def gf_inv(a: int) -> int:
    if a == 0:
        raise ZeroDivisionError("GF(256) inverse of zero")
    return GF_EXP[255 - GF_LOG[a]]


def xor_scaled(dest: bytearray, src: bytes, coef: int) -> None:
    if coef == 0:
        return
    if coef == 1:
        for i in range(len(src)):
            dest[i] ^= src[i]
        return
    mul_row = GF_MUL[coef]
    for i in range(len(src)):
        dest[i] ^= mul_row[src[i]]


def rgb_to_ycbcr420(rgb: bytes, width: int, height: int) -> Tuple[bytearray, bytearray, bytearray]:
    n = width * height
    y_plane = bytearray(n)
    cb_full = bytearray(n)
    cr_full = bytearray(n)

    for p in range(n):
        r = rgb[3 * p + 0]
        g = rgb[3 * p + 1]
        b = rgb[3 * p + 2]

        # Integer approximation of BT.601 full-range conversion.
        y = (77 * r + 150 * g + 29 * b) >> 8
        cb = 128 + ((-43 * r - 85 * g + 128 * b) >> 8)
        cr = 128 + ((128 * r - 107 * g - 21 * b) >> 8)

        y_plane[p] = clip_u8(y)
        cb_full[p] = clip_u8(cb)
        cr_full[p] = clip_u8(cr)

    cw = width // 2
    ch = height // 2
    cb420 = bytearray(cw * ch)
    cr420 = bytearray(cw * ch)

    for yy in range(ch):
        y0 = yy * 2
        row0 = y0 * width
        row1 = (y0 + 1) * width
        out_row = yy * cw
        for xx in range(cw):
            x0 = xx * 2
            i00 = row0 + x0
            i01 = i00 + 1
            i10 = row1 + x0
            i11 = i10 + 1
            cb420[out_row + xx] = (cb_full[i00] + cb_full[i01] + cb_full[i10] + cb_full[i11] + 2) // 4
            cr420[out_row + xx] = (cr_full[i00] + cr_full[i01] + cr_full[i10] + cr_full[i11] + 2) // 4

    return y_plane, cb420, cr420


def ycbcr420_to_rgb(y_plane: bytes, cb420: bytes, cr420: bytes, width: int, height: int) -> bytes:
    cw = width // 2
    rgb = bytearray(width * height * 3)

    for yy in range(height):
        y_row = yy * width
        c_row = (yy // 2) * cw
        for xx in range(width):
            p = y_row + xx
            c = c_row + (xx // 2)

            yv = y_plane[p]
            cb = cb420[c] - 128
            cr = cr420[c] - 128

            r = yv + ((359 * cr) >> 8)
            g = yv - ((88 * cb + 183 * cr) >> 8)
            b = yv + ((454 * cb) >> 8)

            rgb[3 * p + 0] = clip_u8(r)
            rgb[3 * p + 1] = clip_u8(g)
            rgb[3 * p + 2] = clip_u8(b)

    return bytes(rgb)


def block_mean_rect(plane: bytes, stride: int, x0: int, y0: int, block_w: int, block_h: int) -> int:
    total = 0
    base = y0 * stride + x0
    for dy in range(block_h):
        row = base + dy * stride
        for dx in range(block_w):
            total += plane[row + dx]
    return (total + (block_w * block_h // 2)) // (block_w * block_h)


def block_mean(plane: bytes, stride: int, x0: int, y0: int) -> int:
    return block_mean_rect(plane, stride, x0, y0, 4, 4)


def pack_6bit_values(values: Sequence[int]) -> bytes:
    if (len(values) % 4) != 0:
        raise ValueError("6-bit pack expects value count multiple of 4")

    payload = bytearray((len(values) * 6) // 8)
    out = 0
    for i in range(0, len(values), 4):
        v0 = values[i + 0] & 0x3F
        v1 = values[i + 1] & 0x3F
        v2 = values[i + 2] & 0x3F
        v3 = values[i + 3] & 0x3F

        payload[out + 0] = (v0 << 2) | (v1 >> 4)
        payload[out + 1] = ((v1 & 0x0F) << 4) | (v2 >> 2)
        payload[out + 2] = ((v2 & 0x03) << 6) | v3
        out += 3

    return bytes(payload)


def unpack_6bit_values(payload: bytes, count: int) -> List[int]:
    if (count % 4) != 0:
        raise ValueError("6-bit unpack expects value count multiple of 4")
    expected = (count * 6) // 8
    if len(payload) != expected:
        raise ValueError("Unexpected payload size for 6-bit unpack")

    out = [0] * count
    src = 0
    for i in range(0, count, 4):
        b0 = payload[src + 0]
        b1 = payload[src + 1]
        b2 = payload[src + 2]

        out[i + 0] = b0 >> 2
        out[i + 1] = ((b0 & 0x03) << 4) | (b1 >> 4)
        out[i + 2] = ((b1 & 0x0F) << 2) | (b2 >> 6)
        out[i + 3] = b2 & 0x3F
        src += 3

    return out


def pack_variable_bits(values: Sequence[int], bit_widths: Sequence[int]) -> bytes:
    if len(values) != len(bit_widths):
        raise ValueError("values and bit_widths must have identical lengths")

    total_bits = sum(bit_widths)
    out = bytearray((total_bits + 7) // 8)
    bit_pos = 0

    for value, width in zip(values, bit_widths):
        if width <= 0:
            raise ValueError("bit width must be positive")
        max_value = (1 << width) - 1
        if value < 0 or value > max_value:
            raise ValueError("value exceeds declared bit width")

        for shift in range(width - 1, -1, -1):
            if value & (1 << shift):
                out[bit_pos // 8] |= 1 << (7 - (bit_pos % 8))
            bit_pos += 1

    return bytes(out)


def unpack_variable_bits(payload: bytes, bit_widths: Sequence[int]) -> List[int]:
    total_bits = sum(bit_widths)
    if len(payload) * 8 != total_bits:
        raise ValueError("payload size does not match requested bit widths")

    out: List[int] = []
    bit_pos = 0
    for width in bit_widths:
        if width <= 0:
            raise ValueError("bit width must be positive")

        value = 0
        for _ in range(width):
            bit = (payload[bit_pos // 8] >> (7 - (bit_pos % 8))) & 1
            value = (value << 1) | bit
            bit_pos += 1
        out.append(value)

    return out


def pack_y_indices(y_idx: Sequence[int], payload: bytearray) -> None:
    for g in range(12):
        i = 4 * g
        y0 = y_idx[i + 0] & 0x3F
        y1 = y_idx[i + 1] & 0x3F
        y2 = y_idx[i + 2] & 0x3F
        y3 = y_idx[i + 3] & 0x3F
        payload[3 * g + 0] = (y0 << 2) | (y1 >> 4)
        payload[3 * g + 1] = ((y1 & 0x0F) << 4) | (y2 >> 2)
        payload[3 * g + 2] = ((y2 & 0x03) << 6) | y3


def unpack_y_indices(payload: bytes) -> List[int]:
    y_idx = [0] * Y_BLOCKS_PER_TILE
    for g in range(12):
        i = 4 * g
        b0 = payload[3 * g + 0]
        b1 = payload[3 * g + 1]
        b2 = payload[3 * g + 2]
        y_idx[i + 0] = b0 >> 2
        y_idx[i + 1] = ((b0 & 0x03) << 4) | (b1 >> 4)
        y_idx[i + 2] = ((b1 & 0x0F) << 2) | (b2 >> 6)
        y_idx[i + 3] = b2 & 0x3F
    return y_idx


def pack_chroma_indices(cb_idx: Sequence[int], cr_idx: Sequence[int], payload: bytearray) -> None:
    for k in range(6):
        payload[36 + k] = ((cb_idx[2 * k] & 0x0F) << 4) | (cb_idx[2 * k + 1] & 0x0F)
        payload[42 + k] = ((cr_idx[2 * k] & 0x0F) << 4) | (cr_idx[2 * k + 1] & 0x0F)


def unpack_chroma_indices(payload: bytes) -> Tuple[List[int], List[int]]:
    cb_idx = [0] * C_BLOCKS_PER_TILE
    cr_idx = [0] * C_BLOCKS_PER_TILE
    for k in range(6):
        b_cb = payload[36 + k]
        b_cr = payload[42 + k]
        cb_idx[2 * k] = b_cb >> 4
        cb_idx[2 * k + 1] = b_cb & 0x0F
        cr_idx[2 * k] = b_cr >> 4
        cr_idx[2 * k + 1] = b_cr & 0x0F
    return cb_idx, cr_idx


def _encode_tile_fixed(
    profile: CodecProfile,
    y_plane: bytes,
    cb420: bytes,
    cr420: bytes,
    tile_x: int,
    tile_y: int,
    width: int,
) -> bytes:
    payload = bytearray(PAYLOAD_SIZE)

    y0 = tile_y * profile.tile_h
    x0 = tile_x * profile.tile_w

    y_idx: List[int] = []
    for by in range(profile.tile_h // profile.y_block_h):
        py = y0 + by * profile.y_block_h
        for bx in range(profile.tile_w // profile.y_block_w):
            px = x0 + bx * profile.y_block_w
            y_idx.append(quantize_u8(block_mean_rect(y_plane, width, px, py, profile.y_block_w, profile.y_block_h), 64))

    c_tile_w = profile.tile_w // 2
    c_tile_h = profile.tile_h // 2
    c_width = width // 2
    cy0 = tile_y * c_tile_h
    cx0 = tile_x * c_tile_w

    cb_idx: List[int] = []
    cr_idx: List[int] = []
    for by in range(c_tile_h // profile.c_block_h):
        py = cy0 + by * profile.c_block_h
        for bx in range(c_tile_w // profile.c_block_w):
            px = cx0 + bx * profile.c_block_w
            cb_idx.append(quantize_u8(block_mean_rect(cb420, c_width, px, py, profile.c_block_w, profile.c_block_h), 16))
            cr_idx.append(quantize_u8(block_mean_rect(cr420, c_width, px, py, profile.c_block_w, profile.c_block_h), 16))

    pack_y_indices(y_idx, payload)
    pack_chroma_indices(cb_idx, cr_idx, payload)
    return bytes(payload)


def write_constant_block_rect(
    plane: bytearray,
    stride: int,
    x0: int,
    y0: int,
    block_w: int,
    block_h: int,
    value: int,
) -> None:
    base = y0 * stride + x0
    for dy in range(block_h):
        row = base + dy * stride
        for dx in range(block_w):
            plane[row + dx] = value


def write_constant_block(plane: bytearray, stride: int, x0: int, y0: int, value: int) -> None:
    write_constant_block_rect(plane, stride, x0, y0, 4, 4, value)


def decode_tile_payload_p100(
    payload: bytes,
    tile_x: int,
    tile_y: int,
    y_plane: bytearray,
    cb420: bytearray,
    cr420: bytearray,
    width: int,
    height: int,
) -> None:
    del height
    y_idx = unpack_y_indices(payload)
    cb_idx, cr_idx = unpack_chroma_indices(payload)

    y0 = tile_y * TILE_H_P100
    x0 = tile_x * TILE_W_P100

    q = 0
    for by in range(TILE_H_P100 // Y_BLOCK_H_P100):
        py = y0 + by * Y_BLOCK_H_P100
        for bx in range(TILE_W_P100 // Y_BLOCK_W_P100):
            px = x0 + bx * Y_BLOCK_W_P100
            y_val = dequantize_u8(y_idx[q], 64)
            write_constant_block(y_plane, width, px, py, y_val)
            q += 1

    c_width = width // 2
    cy0 = tile_y * C_TILE_H
    cx0 = tile_x * C_TILE_W

    q = 0
    for by in range(C_TILE_H // C_BLOCK_H_P100):
        py = cy0 + by * C_BLOCK_H_P100
        for bx in range(C_TILE_W // C_BLOCK_W_P100):
            px = cx0 + bx * C_BLOCK_W_P100
            cb_val = dequantize_u8(cb_idx[q], 16)
            cr_val = dequantize_u8(cr_idx[q], 16)
            write_constant_block(cb420, c_width, px, py, cb_val)
            write_constant_block(cr420, c_width, px, py, cr_val)
            q += 1


def decode_tile_payload_p200(
    payload: bytes,
    tile_x: int,
    tile_y: int,
    y_plane: bytearray,
    cb420: bytearray,
    cr420: bytearray,
    width: int,
    height: int,
) -> None:
    del height

    y_idx = unpack_y_indices(payload)
    cb_idx, cr_idx = unpack_chroma_indices(payload)

    y0 = tile_y * TILE_H_P200
    x0 = tile_x * TILE_W_P200

    q = 0
    for by in range(TILE_H_P200 // Y_BLOCK_H_P200):
        py = y0 + by * Y_BLOCK_H_P200
        for bx in range(TILE_W_P200 // Y_BLOCK_W_P200):
            px = x0 + bx * Y_BLOCK_W_P200
            y_val = dequantize_u8(y_idx[q], 64)
            write_constant_block_rect(y_plane, width, px, py, Y_BLOCK_W_P200, Y_BLOCK_H_P200, y_val)
            q += 1

    c_width = width // 2
    cy0 = tile_y * C_TILE_H_P200
    cx0 = tile_x * C_TILE_W_P200

    q = 0
    for by in range(C_TILE_H_P200 // C_BLOCK_H_P200):
        py = cy0 + by * C_BLOCK_H_P200
        for bx in range(C_TILE_W_P200 // C_BLOCK_W_P200):
            px = cx0 + bx * C_BLOCK_W_P200
            cb_val = dequantize_u8(cb_idx[q], 16)
            cr_val = dequantize_u8(cr_idx[q], 16)
            write_constant_block_rect(cb420, c_width, px, py, C_BLOCK_W_P200, C_BLOCK_H_P200, cb_val)
            write_constant_block_rect(cr420, c_width, px, py, C_BLOCK_W_P200, C_BLOCK_H_P200, cr_val)
            q += 1


def _encode_tile_p240(
    y_plane: bytes,
    cb420: bytes,
    cr420: bytes,
    tile_x: int,
    tile_y: int,
    width: int,
    height: int,
) -> bytes:
    del height

    y0 = tile_y * TILE_H_P240
    x0 = tile_x * TILE_W_P240

    y_mean: List[int] = []
    for by in range(TILE_H_P240 // Y_BLOCK_H_P240):
        py = y0 + by * Y_BLOCK_H_P240
        for bx in range(TILE_W_P240 // Y_BLOCK_W_P240):
            px = x0 + bx * Y_BLOCK_W_P240
            y_mean.append(block_mean_rect(y_plane, width, px, py, Y_BLOCK_W_P240, Y_BLOCK_H_P240))

    c_width = width // 2
    cy0 = tile_y * C_TILE_H_P240
    cx0 = tile_x * C_TILE_W_P240

    cb_idx: List[int] = []
    cr_idx: List[int] = []
    for by in range(C_TILE_H_P240 // C_BLOCK_H_P240):
        py = cy0 + by * C_BLOCK_H_P240
        for bx in range(C_TILE_W_P240 // C_BLOCK_W_P240):
            px = cx0 + bx * C_BLOCK_W_P240
            cb_mean = block_mean_rect(cb420, c_width, px, py, C_BLOCK_W_P240, C_BLOCK_H_P240)
            cr_mean = block_mean_rect(cr420, c_width, px, py, C_BLOCK_W_P240, C_BLOCK_H_P240)
            cb_idx.append(quantize_u8(cb_mean, 16))
            cr_idx.append(quantize_u8(cr_mean, 16))

    values: List[int] = []
    for idx in Y_HI_INDICES_P240:
        values.append(quantize_u8(y_mean[idx], 64))
    for idx in Y_LO_INDICES_P240:
        values.append(quantize_u8(y_mean[idx], 32))
    values.extend(cb_idx)
    values.extend(cr_idx)

    payload = pack_variable_bits(values, P240_PAYLOAD_BIT_WIDTHS)
    if len(payload) != PAYLOAD_SIZE:
        raise RuntimeError("p240 tile payload must be exactly 48 bytes")
    return payload


def decode_tile_payload_p240(
    payload: bytes,
    tile_x: int,
    tile_y: int,
    y_plane: bytearray,
    cb420: bytearray,
    cr420: bytearray,
    width: int,
    height: int,
) -> None:
    del height

    values = unpack_variable_bits(payload, P240_PAYLOAD_BIT_WIDTHS)
    y_idx_hi = values[:Y_HI_BITS_BLOCKS_P240]
    y_idx_lo = values[Y_HI_BITS_BLOCKS_P240 : Y_HI_BITS_BLOCKS_P240 + Y_LO_BITS_BLOCKS_P240]

    chroma_start = Y_HI_BITS_BLOCKS_P240 + Y_LO_BITS_BLOCKS_P240
    cb_idx = values[chroma_start : chroma_start + C_BLOCKS_PER_TILE_P240]
    cr_idx = values[chroma_start + C_BLOCKS_PER_TILE_P240 :]

    y_vals = [0] * Y_BLOCKS_PER_TILE_P240
    for i, idx in enumerate(Y_HI_INDICES_P240):
        y_vals[idx] = dequantize_u8(y_idx_hi[i], 64)
    for i, idx in enumerate(Y_LO_INDICES_P240):
        y_vals[idx] = dequantize_u8(y_idx_lo[i], 32)

    y0 = tile_y * TILE_H_P240
    x0 = tile_x * TILE_W_P240

    q = 0
    for by in range(TILE_H_P240 // Y_BLOCK_H_P240):
        py = y0 + by * Y_BLOCK_H_P240
        for bx in range(TILE_W_P240 // Y_BLOCK_W_P240):
            px = x0 + bx * Y_BLOCK_W_P240
            write_constant_block_rect(y_plane, width, px, py, Y_BLOCK_W_P240, Y_BLOCK_H_P240, y_vals[q])
            q += 1

    c_width = width // 2
    cy0 = tile_y * C_TILE_H_P240
    cx0 = tile_x * C_TILE_W_P240

    q = 0
    for by in range(C_TILE_H_P240 // C_BLOCK_H_P240):
        py = cy0 + by * C_BLOCK_H_P240
        for bx in range(C_TILE_W_P240 // C_BLOCK_W_P240):
            px = cx0 + bx * C_BLOCK_W_P240
            cb_val = dequantize_u8(cb_idx[q], 16)
            cr_val = dequantize_u8(cr_idx[q], 16)
            write_constant_block_rect(cb420, c_width, px, py, C_BLOCK_W_P240, C_BLOCK_H_P240, cb_val)
            write_constant_block_rect(cr420, c_width, px, py, C_BLOCK_W_P240, C_BLOCK_H_P240, cr_val)
            q += 1


def _encode_tile_p300(
    y_plane: bytes,
    cb420: bytes,
    cr420: bytes,
    tile_x: int,
    tile_y: int,
    width: int,
    height: int,
) -> bytes:
    del height

    y0 = tile_y * TILE_H_P300
    x0 = tile_x * TILE_W_P300

    y_idx: List[int] = []
    for by in range(TILE_H_P300 // Y_BLOCK_H_HD):
        py = y0 + by * Y_BLOCK_H_HD
        for bx in range(TILE_W_P300 // Y_BLOCK_W_P300):
            px = x0 + bx * Y_BLOCK_W_P300
            mean = block_mean_rect(y_plane, width, px, py, Y_BLOCK_W_P300, Y_BLOCK_H_HD)
            y_idx.append(quantize_u8(mean, 64))

    c_width = width // 2
    cy0 = tile_y * C_TILE_H_P300
    cx0 = tile_x * C_TILE_W_P300

    cb_idx: List[int] = []
    cr_idx: List[int] = []
    for by in range(C_TILE_H_P300 // C_BLOCK_H_HD):
        py = cy0 + by * C_BLOCK_H_HD
        for bx in range(C_TILE_W_P300 // C_BLOCK_W_HD):
            px = cx0 + bx * C_BLOCK_W_HD
            cb_mean = block_mean_rect(cb420, c_width, px, py, C_BLOCK_W_HD, C_BLOCK_H_HD)
            cr_mean = block_mean_rect(cr420, c_width, px, py, C_BLOCK_W_HD, C_BLOCK_H_HD)
            cb_idx.append(quantize_u8(cb_mean, 64))
            cr_idx.append(quantize_u8(cr_mean, 64))

    values = y_idx + cb_idx + cr_idx
    payload = pack_6bit_values(values)
    if len(payload) != PAYLOAD_SIZE:
        raise RuntimeError("HD tile payload must be exactly 48 bytes")
    return payload


def encode_tile_payload(
    profile: CodecProfile,
    y_plane: bytes,
    cb420: bytes,
    cr420: bytes,
    tile_x: int,
    tile_y: int,
    width: int,
    height: int,
) -> bytes:
    if profile.name == PROFILE_P100.name:
        return _encode_tile_fixed(profile, y_plane, cb420, cr420, tile_x, tile_y, width)
    if profile.name == PROFILE_P200.name:
        return _encode_tile_fixed(profile, y_plane, cb420, cr420, tile_x, tile_y, width)
    if profile.name == PROFILE_P240.name:
        return _encode_tile_p240(y_plane, cb420, cr420, tile_x, tile_y, width, height)
    if profile.name == PROFILE_P300.name:
        return _encode_tile_p300(y_plane, cb420, cr420, tile_x, tile_y, width, height)
    raise ValueError(f"Unsupported codec profile: {profile.name}")


def decode_tile_payload_hd(
    payload: bytes,
    tile_x: int,
    tile_y: int,
    y_plane: bytearray,
    cb420: bytearray,
    cr420: bytearray,
    width: int,
    height: int,
) -> None:
    del height

    values = unpack_6bit_values(payload, Y_BLOCKS_PER_TILE_HD + C_BLOCKS_PER_TILE_HD + C_BLOCKS_PER_TILE_HD)
    y_idx = values[:Y_BLOCKS_PER_TILE_HD]
    cb_idx = values[Y_BLOCKS_PER_TILE_HD : Y_BLOCKS_PER_TILE_HD + C_BLOCKS_PER_TILE_HD]
    cr_idx = values[Y_BLOCKS_PER_TILE_HD + C_BLOCKS_PER_TILE_HD :]

    y0 = tile_y * TILE_H_P300
    x0 = tile_x * TILE_W_P300

    q = 0
    for by in range(TILE_H_P300 // Y_BLOCK_H_HD):
        py = y0 + by * Y_BLOCK_H_HD
        for bx in range(TILE_W_P300 // Y_BLOCK_W_P300):
            px = x0 + bx * Y_BLOCK_W_P300
            y_val = dequantize_u8(y_idx[q], 64)
            write_constant_block_rect(y_plane, width, px, py, Y_BLOCK_W_P300, Y_BLOCK_H_HD, y_val)
            q += 1

    c_width = width // 2
    cy0 = tile_y * C_TILE_H_P300
    cx0 = tile_x * C_TILE_W_P300

    q = 0
    for by in range(C_TILE_H_P300 // C_BLOCK_H_HD):
        py = cy0 + by * C_BLOCK_H_HD
        for bx in range(C_TILE_W_P300 // C_BLOCK_W_HD):
            px = cx0 + bx * C_BLOCK_W_HD
            cb_val = dequantize_u8(cb_idx[q], 64)
            cr_val = dequantize_u8(cr_idx[q], 64)
            write_constant_block_rect(cb420, c_width, px, py, C_BLOCK_W_HD, C_BLOCK_H_HD, cb_val)
            write_constant_block_rect(cr420, c_width, px, py, C_BLOCK_W_HD, C_BLOCK_H_HD, cr_val)
            q += 1


def copy_rect(
    plane: bytearray,
    stride: int,
    src_x: int,
    src_y: int,
    dst_x: int,
    dst_y: int,
    rect_w: int,
    rect_h: int,
) -> None:
    for row in range(rect_h):
        src_off = (src_y + row) * stride + src_x
        dst_off = (dst_y + row) * stride + dst_x
        plane[dst_off : dst_off + rect_w] = plane[src_off : src_off + rect_w]


def nearest_known_sources(known: Sequence[bool], tiles_x: int, tiles_y: int) -> List[Optional[int]]:
    total = tiles_x * tiles_y
    if len(known) != total:
        raise ValueError("known mask size mismatch")

    source: List[Optional[int]] = [None] * total
    visited = [False] * total
    q: deque[int] = deque()

    for tile_id, is_known in enumerate(known):
        if is_known:
            source[tile_id] = tile_id
            visited[tile_id] = True
            q.append(tile_id)

    if not q:
        return source

    while q:
        tile_id = q.popleft()
        tx = tile_id % tiles_x
        ty = tile_id // tiles_x

        neighbors = ((tx - 1, ty), (tx + 1, ty), (tx, ty - 1), (tx, ty + 1))
        for nx, ny in neighbors:
            if nx < 0 or nx >= tiles_x or ny < 0 or ny >= tiles_y:
                continue

            nid = ny * tiles_x + nx
            if visited[nid]:
                continue

            visited[nid] = True
            source[nid] = source[tile_id]
            q.append(nid)

    return source


def conceal_missing_tiles(
    payloads: Sequence[Optional[bytes]],
    profile: CodecProfile,
    y_plane: bytearray,
    cb420: bytearray,
    cr420: bytearray,
    width: int,
) -> None:
    known = [payload is not None for payload in payloads]
    if all(known) or not any(known):
        return

    sources = nearest_known_sources(known, profile.tiles_x, profile.tiles_y)

    tile_w = profile.tile_w
    tile_h = profile.tile_h
    c_tile_w = tile_w // 2
    c_tile_h = tile_h // 2
    c_width = width // 2

    for dst_id, is_known in enumerate(known):
        if is_known:
            continue

        src_id = sources[dst_id]
        if src_id is None:
            continue

        src_tx = src_id % profile.tiles_x
        src_ty = src_id // profile.tiles_x
        dst_tx = dst_id % profile.tiles_x
        dst_ty = dst_id // profile.tiles_x

        src_x = src_tx * tile_w
        src_y = src_ty * tile_h
        dst_x = dst_tx * tile_w
        dst_y = dst_ty * tile_h

        copy_rect(y_plane, width, src_x, src_y, dst_x, dst_y, tile_w, tile_h)

        src_cx = src_tx * c_tile_w
        src_cy = src_ty * c_tile_h
        dst_cx = dst_tx * c_tile_w
        dst_cy = dst_ty * c_tile_h

        copy_rect(cb420, c_width, src_cx, src_cy, dst_cx, dst_cy, c_tile_w, c_tile_h)
        copy_rect(cr420, c_width, src_cx, src_cy, dst_cx, dst_cy, c_tile_w, c_tile_h)


def decode_image_from_data_payloads(
    payloads: Sequence[Optional[bytes]],
    width: int,
    height: int,
    profile: CodecProfile,
    conceal_missing: bool = False,
) -> bytes:
    y_plane = bytearray([128] * (width * height))
    c_size = (width // 2) * (height // 2)
    cb420 = bytearray([128] * c_size)
    cr420 = bytearray([128] * c_size)

    for tile_id, payload in enumerate(payloads):
        if payload is None:
            continue
        tx = tile_id % profile.tiles_x
        ty = tile_id // profile.tiles_x

        if profile.name == PROFILE_P100.name:
            decode_tile_payload_p100(payload, tx, ty, y_plane, cb420, cr420, width, height)
        elif profile.name == PROFILE_P200.name:
            decode_tile_payload_p200(payload, tx, ty, y_plane, cb420, cr420, width, height)
        elif profile.name == PROFILE_P240.name:
            decode_tile_payload_p240(payload, tx, ty, y_plane, cb420, cr420, width, height)
        elif profile.name == PROFILE_P300.name:
            decode_tile_payload_hd(payload, tx, ty, y_plane, cb420, cr420, width, height)
        else:
            raise ValueError(f"Unsupported codec profile: {profile.name}")

    if conceal_missing:
        conceal_missing_tiles(payloads, profile, y_plane, cb420, cr420, width)

    return ycbcr420_to_rgb(y_plane, cb420, cr420, width, height)


def coefficient_row(frame_id: int, symbol_id: int, k_data: int, coeff_seed: int) -> bytearray:
    if symbol_id < k_data:
        row = bytearray(k_data)
        row[symbol_id] = 1
        return row

    seed = (coeff_seed ^ (frame_id * 0x9E3779B1) ^ (symbol_id * 0x85EBCA77)) & 0xFFFFFFFF
    rng = random.Random(seed)
    row = bytearray(rng.getrandbits(8) for _ in range(k_data))

    if not any(row):
        row[symbol_id % k_data] = 1

    return row


def combine_data_payloads(coeff: bytes, data_payloads: Sequence[bytes]) -> bytes:
    out = bytearray(len(data_payloads[0]))
    for i, c in enumerate(coeff):
        if c == 0:
            continue
        xor_scaled(out, data_payloads[i], c)
    return bytes(out)


def solve_full_rank(
    rows: List[bytearray],
    values: List[bytearray],
    variables: int,
) -> Tuple[Optional[List[bytes]], int]:
    if variables == 0:
        return [], 0

    m = len(rows)
    pivot_row_for_col = [-1] * variables
    r = 0

    for c in range(variables):
        pivot = -1
        for rr in range(r, m):
            if rows[rr][c] != 0:
                pivot = rr
                break

        if pivot < 0:
            continue

        if pivot != r:
            rows[r], rows[pivot] = rows[pivot], rows[r]
            values[r], values[pivot] = values[pivot], values[r]

        pivot_coef = rows[r][c]
        if pivot_coef != 1:
            inv = gf_inv(pivot_coef)
            mul_row = GF_MUL[inv]
            row_r = rows[r]
            val_r = values[r]
            for cc in range(c, variables):
                row_r[cc] = mul_row[row_r[cc]]
            for b in range(len(val_r)):
                val_r[b] = mul_row[val_r[b]]

        row_r = rows[r]
        val_r = values[r]
        for rr in range(m):
            if rr == r:
                continue
            factor = rows[rr][c]
            if factor == 0:
                continue
            mul_row = GF_MUL[factor]
            row_i = rows[rr]
            val_i = values[rr]
            for cc in range(c, variables):
                row_i[cc] ^= mul_row[row_r[cc]]
            for b in range(len(val_r)):
                val_i[b] ^= mul_row[val_r[b]]

        pivot_row_for_col[c] = r
        r += 1
        if r == variables:
            break

    rank = r
    if rank < variables:
        return None, rank

    solution = [b"" for _ in range(variables)]
    for col in range(variables):
        pr = pivot_row_for_col[col]
        if pr < 0:
            return None, rank
        solution[col] = bytes(values[pr])

    return solution, rank


def recover_data_payloads(
    received: Sequence[Packet],
    frame_id: int,
    k_data: int,
    coeff_seed: int = COEFF_SEED,
) -> Tuple[List[Optional[bytes]], bool, int, int]:
    data: List[Optional[bytes]] = [None] * k_data
    for pkt in received:
        if pkt.symbol_id < k_data:
            data[pkt.symbol_id] = pkt.payload

    missing = [i for i, payload in enumerate(data) if payload is None]
    missing_count = len(missing)
    if missing_count == 0:
        return data, True, 0, 0

    unknown_col: Dict[int, int] = {tile_id: col for col, tile_id in enumerate(missing)}

    rows: List[bytearray] = []
    rhs_values: List[bytearray] = []

    for pkt in received:
        coeff_full = coefficient_row(frame_id, pkt.symbol_id, k_data, coeff_seed)
        row = bytearray(missing_count)
        rhs = bytearray(pkt.payload)

        for tile_id, coeff in enumerate(coeff_full):
            if coeff == 0:
                continue
            known = data[tile_id]
            if known is None:
                col = unknown_col[tile_id]
                row[col] = coeff
            else:
                xor_scaled(rhs, known, coeff)

        if any(row):
            rows.append(row)
            rhs_values.append(rhs)

    if not rows:
        return data, False, missing_count, 0

    solved, rank = solve_full_rank(rows, rhs_values, missing_count)
    if solved is None:
        return data, False, missing_count, rank

    for col, payload in enumerate(solved):
        tile_id = missing[col]
        data[tile_id] = payload

    return data, True, missing_count, rank














class MeshCamCodec:
    """Stateful codec wrapper exposing packet/image encode-decode and FEC helpers."""

    def __init__(self, width: int = WIDTH, height: int = HEIGHT, coeff_seed: int = COEFF_SEED) -> None:
        # Current profiles are precomputed for 320x240; fail fast on unsupported geometry.
        if width != WIDTH or height != HEIGHT:
            raise ValueError(f"Only {WIDTH}x{HEIGHT} is currently supported")

        self.width = width
        self.height = height
        self.coeff_seed = coeff_seed
        self.payload_size = PAYLOAD_SIZE
        self.profiles: Dict[str, CodecProfile] = CODEC_PROFILES

    def get_profile(self, name: str) -> CodecProfile:
        if name not in self.profiles:
            raise KeyError(f"Unknown profile: {name}")
        return self.profiles[name]

    def profile_names(self) -> List[str]:
        return sorted(self.profiles.keys())


    def load_image_rgb(self, path: str) -> bytes:
        if not PIL_AVAILABLE:
            raise RuntimeError("Pillow is required for --input images. Install with: pip install pillow")

        img = Image.open(path).convert("RGB")
        if img.size != (self.width, self.height):
            img = img.resize((self.width, self.height), Image.Resampling.BICUBIC)
        return img.tobytes()

    def save_ppm(self, path: str, rgb: bytes, width: int, height: int) -> None:
        header = f"P6\n{width} {height}\n255\n".encode("ascii")
        with open(path, "wb") as f:
            f.write(header)
            f.write(rgb)

    def save_image(self, path: str, rgb: bytes) -> None:
        suffix = os.path.splitext(path)[1].lower()
        if PIL_AVAILABLE and suffix in {".png", ".jpg", ".jpeg", ".bmp"}:
            img = Image.frombytes("RGB", (self.width, self.height), rgb)
            img.save(path)
            return

        ppm_path = path if suffix == ".ppm" else os.path.splitext(path)[0] + ".ppm"
        self.save_ppm(ppm_path, rgb, self.width, self.height)


    def encode_image_to_data_payloads(self, rgb: bytes, profile: CodecProfile) -> List[bytes]:
        y_plane, cb420, cr420 = rgb_to_ycbcr420(rgb, self.width, self.height)
        payloads: List[bytes] = []

        for tile_id in range(profile.data_tiles):
            tx = tile_id % profile.tiles_x
            ty = tile_id // profile.tiles_x
            payload = encode_tile_payload(profile, y_plane, cb420, cr420, tx, ty, self.width, self.height)

            payloads.append(payload)

        return payloads
    def decode_image_from_data_payloads(
        self,
        payloads: Sequence[Optional[bytes]],
        profile: CodecProfile,
        conceal_missing: bool = False,
    ) -> bytes:
        return decode_image_from_data_payloads(
            payloads,
            self.width,
            self.height,
            profile,
            conceal_missing=conceal_missing,
        )

    def build_packets(self, data_payloads: Sequence[bytes], frame_id: int, repair_count: int, metadata: str = "") -> List[Packet]:
        k_data = len(data_payloads)
        n_total = k_data + repair_count

        extended: List[bytes] = [
            data_payloads[sid] + bytes([ord(metadata[sid]) if sid < len(metadata) else 0])
            for sid in range(k_data)
        ]

        packets: List[Packet] = []
        for sid in range(k_data):
            packets.append(
                Packet(
                    frame_id=frame_id,
                    symbol_id=sid,
                    k_data=k_data,
                    n_total=n_total,
                    is_repair=False,
                    payload=extended[sid],
                )
            )

        for sid in range(k_data, n_total):
            coeff = coefficient_row(frame_id, sid, k_data, self.coeff_seed)
            payload = combine_data_payloads(coeff, extended)
            packets.append(
                Packet(
                    frame_id=frame_id,
                    symbol_id=sid,
                    k_data=k_data,
                    n_total=n_total,
                    is_repair=True,
                    payload=payload,
                )
            )

        return packets

    def recover_data_payloads(
        self,
        received: Sequence[Packet],
        frame_id: int,
        k_data: int,
    ) -> Tuple[List[Optional[bytes]], bool, int, int]:
        return recover_data_payloads(received, frame_id, k_data, coeff_seed=self.coeff_seed)


    def serialize_packet_dump_record(self, packet: Packet) -> bytes:
        if len(packet.payload) != PAYLOAD_SIZE:
            raise ValueError("packet payload has unexpected size")

        for field_name, field_value in (
            ("frame_id", packet.frame_id),
            ("symbol_id", packet.symbol_id),
            ("k_data", packet.k_data),
            ("n_total", packet.n_total),
        ):
            if field_value < 0 or field_value > 0xFFFF:
                raise ValueError(f"{field_name} is out of supported dump range 0..65535")

        flags = 0x01 if packet.is_repair else 0x00
        header = bytes(
            [
                flags,
                0x01,
                (packet.frame_id >> 8) & 0xFF,
                packet.frame_id & 0xFF,
                (packet.symbol_id >> 8) & 0xFF,
                packet.symbol_id & 0xFF,
                (packet.k_data >> 8) & 0xFF,
                packet.k_data & 0xFF,
                (packet.n_total >> 8) & 0xFF,
                packet.n_total & 0xFF,
            ]
        )
        return header + packet.payload
    
    def save_packets_bin(self, path: str, packets: Sequence[Packet]) -> None:
        records = [self.serialize_packet_dump_record(packet) for packet in packets]
        record_size = PACKET_DUMP_HEADER_SIZE + PAYLOAD_SIZE

        with open(path, "wb") as f:
            f.write(record_size.to_bytes(2, "big"))
            f.write(len(records).to_bytes(4, "big"))
            for record in records:
                f.write(record)

    def save_packets_hex(self, path: str, packets: Sequence[Packet]) -> None:
        record_size = PACKET_DUMP_HEADER_SIZE + PAYLOAD_SIZE
        with open(path, "w", encoding="ascii") as f:
            f.write(
                f"# record_size={record_size} "
                f"count={len(packets)}\n"
            )
            for i, packet in enumerate(packets):
                record = self.serialize_packet_dump_record(packet)
                packet_type = "R" if packet.is_repair else "D"
                f.write(
                    f"{i:05d} symbol={packet.symbol_id:05d} type={packet_type} "
                    f"{record.hex()}\n"
                )

    def save_packet_dumps(self, base_path: str, packets: Sequence[Packet]) -> Tuple[str, str]:
        stem = os.path.splitext(base_path)[0]
        bin_path = stem + ".bin"
        hex_path = stem + ".hex.txt"
        self.save_packets_bin(bin_path, packets)
        self.save_packets_hex(hex_path, packets)
        return bin_path, hex_path

__all__ = [
    "WIDTH",
    "HEIGHT",
    "CODEC_PROFILES",
    "Packet",
    "CodecProfile",
    "MeshCamCodec",
]
