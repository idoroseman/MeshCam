# MeshCam Codec Internals

This document explains how `meshcam_codec.py` works in detail: data model, color transform, profile layouts, packet construction, FEC recovery, and decode behavior.

## 1. Design Goals

The codec is built for constrained, lossy mesh transport:

- Fixed image size: `320x240`
- Fixed symbol payload: `48 bytes`
- Packet-level independence for data symbols (each data symbol corresponds to one tile)
- Optional repair symbols generated with deterministic RLNC-style mixing over `GF(256)`
- Fast integer math, deterministic behavior, low implementation complexity

The codec core only handles codec/FEC mechanics. Simulation utilities such as packet loss emulation and quality metrics are in `meshcam_codec_sim.py`.

## 2. Core Data Model

### `Packet`

Each packet is represented as:

- `frame_id: int`
- `symbol_id: int`
- `k_data: int` (number of data symbols for this frame)
- `n_total: int` (data + repair)
- `is_repair: bool`
- `payload: bytes` (always `48` bytes)

### `CodecProfile`

Defines tiling geometry per mode:

- `name`
- `tile_w`, `tile_h`
- `tiles_x`, `tiles_y`
- `data_tiles` (`K`)

Profiles are available as `p100`, `p200`, `p240`, `p300`.

## 3. Image and Color Pipeline

### RGB -> YCbCr 4:2:0 (`rgb_to_ycbcr420`)

The codec converts RGB to YCbCr using integer approximations of BT.601 full-range:

- `Y  = ( 77*R + 150*G +  29*B) >> 8`
- `Cb = 128 + ((-43*R -  85*G + 128*B) >> 8)`
- `Cr = 128 + ((128*R - 107*G -  21*B) >> 8)`

Then Cb/Cr are downsampled to 4:2:0 by averaging each 2x2 block with rounding.

### Quantization Helpers

- `quantize_u8(value, levels) = (value * (levels - 1) + 127) // 255`
- `dequantize_u8(index, levels) = (index * 255 + ((levels - 1)//2)) // (levels - 1)`

### YCbCr 4:2:0 -> RGB (`ycbcr420_to_rgb`)

Decode uses integer reconstruction:

- `R = Y + ((359*Cr) >> 8)`
- `G = Y - ((88*Cb + 183*Cr) >> 8)`
- `B = Y + ((454*Cb) >> 8)`

with `Cb`, `Cr` interpreted as signed offsets around 128.

## 4. Profiles and Bit Budgets

All profiles produce exactly `48 bytes = 384 bits` per tile payload.

### `p100`

- Tile grid: `10x10`
- Tile size: `32x24`
- `K = 100`
- Luma sampling: `4x4` blocks -> `48` values, `6 bits` each
- Chroma sampling: Cb/Cr each from `4x4` blocks on 4:2:0 plane -> `12` values each, `4 bits`

Budget:

- Y: `48*6 = 288 bits`
- Cb: `12*4 = 48 bits`
- Cr: `12*4 = 48 bits`
- Total: `384 bits`

### `p200`

- Tile grid: `20x10`
- Tile size: `16x24`
- `K = 200`
- Luma blocks: `4x2` -> `48` values, `6 bits`
- Chroma blocks: `2x4` on 4:2:0 plane -> `12` values each, `4 bits`

Same bit budget as `p100`, but with finer horizontal tile granularity.

### `p240`

- Tile grid: `16x15`
- Tile size: `20x16`
- `K = 240`
- Luma blocks: `4x2` -> `40` values
- Mixed luma precision:
  - `24` selected indices at `6 bits`
  - `16` indices at `5 bits`
- Chroma blocks: `2x2` -> `20` values each at `4 bits`

Budget:

- Y: `24*6 + 16*5 = 224 bits`
- Cb: `20*4 = 80 bits`
- Cr: `20*4 = 80 bits`
- Total: `384 bits`

The set `Y_HI_INDICES_P240` distributes higher-precision luma entries spatially to reduce local bias.

### `p300`

- Tile grid: `20x15`
- Tile size: `16x16`
- `K = 300`
- Luma blocks: `4x2` -> `32` values, `6 bits`
- Chroma blocks: `2x2` -> `16` values each, `6 bits`

Budget:

- `64` values total (`32 + 16 + 16`) x `6 bits` = `384 bits`

## 5. Payload Packing

### Fixed packers (`p100`, `p200`)

- `pack_y_indices` packs 4 x 6-bit values into 3 bytes.
- `pack_chroma_indices` packs two 4-bit values per byte.
- Layout in payload:
  - bytes `0..35`: Y (48 values)
  - bytes `36..41`: Cb (12 values)
  - bytes `42..47`: Cr (12 values)

### Variable bit packer (`p240`)

`pack_variable_bits` writes values with explicit per-value widths from `P240_PAYLOAD_BIT_WIDTHS`.
Order is:

1. high-precision Y values (6-bit)
2. low-precision Y values (5-bit)
3. Cb (4-bit)
4. Cr (4-bit)

### 6-bit generic packer (`p300`)

`pack_6bit_values` packs 4 x 6-bit values into 3 bytes for all 64 values.

## 6. Encoding Flow (Frame Level)

`MeshCamCodec.encode_image_to_data_payloads`:

1. Convert full RGB frame to `Y`, `Cb420`, `Cr420`.
2. For each tile ID:
   - compute tile coordinates `(tx, ty)`
   - call profile-specific tile encoder
3. Return list of `K` payloads.

`MeshCamCodec.build_packets`:

1. Emit systematic packets `symbol_id=0..K-1` using direct tile payloads.
2. Emit repair packets `symbol_id=K..N-1` using linear combinations over `GF(256)`.

## 7. Repair Coefficients and GF(256)

### Finite field setup

The codec precomputes:

- `GF_EXP`, `GF_LOG`
- multiplication table `GF_MUL`

using primitive polynomial `0x11D`.

### Coefficient generation (`coefficient_row`)

For symbol `s`:

- if `s < K`: systematic one-hot row
- else: deterministic pseudo-random row seeded from:
  - `coeff_seed`
  - `frame_id`
  - `symbol_id`

If a generated repair row is all zeros, one coefficient is forced to `1`.

### Payload mixing (`combine_data_payloads`)

Repair payload byte vector is formed by XOR-accumulating `coef * data_payload` over `GF(256)` for all `K` data payloads.

## 8. Recovery Pipeline

`recover_data_payloads(received, frame_id, k_data)`:

1. Fill known data payload slots from received systematic packets.
2. Identify missing data indices.
3. Build linear system only on missing unknowns:
   - subtract known contributions from each equation RHS
   - keep only columns for missing symbols
4. Solve with `solve_full_rank` Gaussian elimination over `GF(256)`.
5. If full rank is achieved, fill missing payloads and report success.

Returns:

- payload list (`List[Optional[bytes]]`)
- solved flag
- missing-count before solve
- matrix rank

## 9. Decoding and Concealment

`decode_image_from_data_payloads`:

1. Initialize neutral planes (`Y=128`, `Cb=Cr=128`).
2. Decode all present tile payloads into planes.
3. Optionally conceal missing tiles (`conceal_missing=True`) by nearest-known tile copy.
4. Convert reconstructed YCbCr420 back to RGB.

### Concealment strategy

`conceal_missing_tiles` uses multi-source BFS (`nearest_known_sources`) across tile grid:

- every known tile is a BFS source
- each missing tile maps to nearest known source (4-neighborhood Manhattan expansion)
- copies Y and chroma tile rectangles from source to destination

This avoids obvious neutral-color holes when FEC cannot fully recover.

## 10. Packet Dump Serialization

`serialize_packet_dump_record` builds each record as:

- 10-byte header
- 48-byte payload

Header bytes:

- byte 0: flags (`bit0=repair`)
- byte 1: version (`1`)
- bytes 2..3: `frame_id`
- bytes 4..5: `symbol_id`
- bytes 6..7: `k_data`
- bytes 8..9: `n_total`

`save_packet_dumps` writes:

- binary (`.bin`) with magic `MCAMPKT1`
- text hex (`.hex.txt`) with one record per line

## 11. `MeshCamCodec` Class API

Core responsibilities:

- profile lookup and defaults
- image load/save (with Pillow when available)
- tile payload encode/decode
- packet construction
- FEC recovery
- packet dump writing

Important behavior:

- constructor enforces only `320x240` currently
- default repair counts:
  - `p100`: 50
  - `p200`: 100
  - `p240`: 60
  - `p300`: 0

## 12. Separation from Simulator

The following are intentionally in `meshcam_codec_sim.py`, not in codec core:

- packet-loss channel simulation
- distortion metrics (`mse`, `psnr`, diff stats)
- synthetic image generator
- absolute diff image helper

This keeps `meshcam_codec.py` focused on codec and FEC logic.

## 13. Complexity Notes

Per frame, dominant work is:

- tile statistics over all blocks
- `GF(256)` linear mixing for repair generation
- Gaussian elimination on missing-symbol system at receiver

At higher loss and larger `K`, recovery solve cost increases, especially near rank-deficient cases.

## 14. Current Limitations

- Geometry is fixed to `320x240`.
- Quantization is intentionally coarse/simple (block means only).
- No entropy coding across symbols.
- Repair coefficient matrix is deterministic pseudo-random, not adaptively optimized.
- Concealment is nearest-tile copy, not texture-aware inpainting.

Despite this, the implementation is deterministic, robust, and easy to reason about for constrained packetized transport.
