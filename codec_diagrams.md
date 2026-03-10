# MeshCam Codec Diagrams

This is a visual companion to `codec.md`.
It explains the same codec using flow diagrams and compact layouts.

## 1. End-to-End Dataflow

```mermaid
flowchart LR
    A[RGB frame 320x240] --> B[rgb_to_ycbcr420]
    B --> C{Profile encode path}

    C -->|p100| D1[encode_tile_payload]
    C -->|p200| D2[encode_tile_payload_p200]
    C -->|p240| D3[encode_tile_payload_p240]
    C -->|p300| D4[encode_tile_payload_hd]

    D1 --> E[data payloads K x 48B]
    D2 --> E
    D3 --> E
    D4 --> E

    E --> F[build_packets]
    F --> G[systematic symbols 0..K-1]
    F --> H[repair symbols K..N-1]

    G --> I[channel]
    H --> I

    I --> J[received packets]
    J --> K[recover_data_payloads]
    K --> L[decode_image_from_data_payloads]
    L --> M[ycbcr420_to_rgb]
    M --> N[decoded RGB frame]
```

## 2. Profile Geometry and Bit Budget

```mermaid
flowchart TB
    P100[p100\nTiles: 10x10\nTile: 32x24\nK=100\nY:48x6\nCb:12x4\nCr:12x4\nTotal 384 bits] --> All[48 bytes payload]
    P200[p200\nTiles: 20x10\nTile: 16x24\nK=200\nY:48x6\nCb:12x4\nCr:12x4\nTotal 384 bits] --> All
    P240[p240\nTiles: 16x15\nTile: 20x16\nK=240\nY:24x6 + 16x5\nCb:20x4\nCr:20x4\nTotal 384 bits] --> All
    P300[p300\nTiles: 20x15\nTile: 16x16\nK=300\nY:32x6\nCb:16x6\nCr:16x6\nTotal 384 bits] --> All
```

## 3. Payload Layouts

### p100 / p200 fixed layout

- bytes `0..35`: luma indices
- bytes `36..41`: Cb indices
- bytes `42..47`: Cr indices

```text
|<------------------------- 48 bytes -------------------------->|
| bytes 0..35 (Y) | bytes 36..41 (Cb) | bytes 42..47 (Cr) |
```

### p240 variable-bit stream

```mermaid
flowchart LR
    A[24 Y values x 6b] --> D[pack_variable_bits]
    B[16 Y values x 5b] --> D
    C[Cb 20 x 4b + Cr 20 x 4b] --> D
    D --> E[384-bit stream = 48 bytes]
```

### p300 uniform 6-bit stream

```mermaid
flowchart LR
    A[64 total values: Y32 + Cb16 + Cr16] --> B[pack_6bit_values]
    B --> C[384 bits = 48 bytes]
```

## 4. Packet Record Layout

Each dumped packet record is 58 bytes:

- 10-byte header
- 48-byte payload

```text
Byte 0   : flags (bit0 = is_repair)
Byte 1   : header version (1)
Byte 2-3 : frame_id (big-endian)
Byte 4-5 : symbol_id (big-endian)
Byte 6-7 : k_data (big-endian)
Byte 8-9 : n_total (big-endian)
Byte 10+ : payload[48]
```

## 5. Repair Symbol Construction

```mermaid
flowchart TD
    A[frame_id, symbol_id, k_data, coeff_seed] --> B[coefficient_row]
    B --> C{symbol_id < k_data?}
    C -->|yes| D[one-hot row]
    C -->|no| E[deterministic random row in GF(256)]
    E --> F{all zeros?}
    F -->|yes| G[force one coeff to 1]
    F -->|no| H[use row as-is]
    D --> I[combine_data_payloads]
    G --> I
    H --> I
    I --> J[repair payload (48B)]
```

## 6. Decoder Recovery (Linear Solve)

```mermaid
flowchart TD
    A[received packets] --> B[fill known data payload slots]
    B --> C[find missing data symbol IDs]
    C --> D[build reduced linear system over missing symbols]
    D --> E[solve_full_rank over GF(256)]
    E --> F{full rank?}
    F -->|yes| G[recover all missing payloads]
    F -->|no| H[partial/failed recovery]
    G --> I[decode_image_from_data_payloads]
    H --> I
    I --> J{conceal_missing?}
    J -->|yes| K[nearest_known_sources BFS copy]
    J -->|no| L[leave neutral tiles]
    K --> M[ycbcr420_to_rgb]
    L --> M
    M --> N[decoded RGB]
```

## 7. Color Path Detail

```mermaid
flowchart LR
    A[RGB] --> B[rgb_to_ycbcr420]
    B --> C[Y plane]
    B --> D[Cb420 plane]
    B --> E[Cr420 plane]
    C --> F[block means + quantize]
    D --> F
    E --> F
    F --> G[payload packing]
    G --> H[payload unpacking]
    H --> I[dequantize + fill blocks]
    I --> J[ycbcr420_to_rgb]
```

## 8. Where Code Lives

- Codec/FEC core: `meshcam_codec.py`
- Simulation helpers (loss, PSNR/MSE, diff image): `meshcam_codec_sim.py`
- Deep narrative explanation: `codec.md`

