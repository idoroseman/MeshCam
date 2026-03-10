# MeshCam Codec + Simulator

This workspace contains:

- `meshcam_codec.py`: codec core implementation, FEC, packet serialization, and helper utilities.
- `meshcam_codec_sim.py`: command-line simulator that uses the codec object.

The system is designed for `320x240` images, fixed `48-byte` packet payloads, and lossy packet-network simulation.

Additional documentation:

- `codec.md`: detailed codec internals explanation.
- `codec_diagrams.md`: diagram-focused visual walkthrough.

## Files

- `meshcam_codec.py`
- `meshcam_codec_sim.py`
- `mesh_send.py`
- `mesh_receive.py`

Optional output folders created by runs:

- `out/`
- `out_compare/`
- `out_*` (various experiments)

## Requirements

Python 3.9+ is recommended.

Optional dependency for loading/saving common image formats (`.jpg`, `.png`):

```bash
pip install pillow
```

Without Pillow, the simulator can still run synthetic input and write `.ppm` outputs.

Optional dependencies for Meshtastic transfer scripts:

```bash
pip install meshtastic pypubsub
```

## Quick Start

Run one simulation with profile `p200`:

```bash
python meshcam_codec_sim.py --profile p200 --input ../aerial_sample.jpg --loss-rate 0.25 --trials 20 --output-format ppm
```

This writes outputs to `out/` by default.

## Meshtastic Transfer Scripts

Two scripts are included for live transfer over Meshtastic nodes connected via serial or BLE:

- `mesh_send.py`: encodes one image and sends codec packets over Meshtastic data messages.
- `mesh_receive.py`: listens for those packets, recovers missing tiles with FEC, and writes a decoded image.

Meshtastic wire format note: the transfer scripts use a compact `p200`-only record format
of `52` bytes per packet (`frame_id[2] + symbol_id[2] + payload[48]`).

### Serial Example

1. Start receiver on the destination host:

The receiver runs continuously by default and decodes each frame it can recover.
Stop with `Ctrl+C`.

```bash
python mesh_receive.py \
  --serial /dev/tty.usbserial-XXXX \
  --output-dir out_mesh_rx \
  --save-packets
```

2. Send one frame from the source host:

```bash
python mesh_send.py \
  --serial /dev/tty.usbserial-YYYY \
  --image ../aerial_sample.jpg \
  --frame-id 123 \
  --port-num 256 \
  --channel-index 0
```

### BLE Example

List available BLE devices:

```bash
python mesh_receive.py --ble
```

```bash
python mesh_send.py --ble
```

```bash
python mesh_send.py \
  --ble AA:BB:CC:DD:EE:FF \
  --image ../aerial_sample.jpg
```

```bash
python mesh_receive.py \
  --ble AA:BB:CC:DD:EE:FF \
  --output-dir out_mesh_rx
```

## Codec Profiles

All profiles use `48-byte` payload packets.

- `p100`: `K=100` data packets
- `p200`: `K=200` data packets
- `p240`: `K=240` data packets
- `p300`: `K=300` data packets

Default repair packet count (when `--repair -1`):

- `p100 -> R=50`
- `p200 -> R=100`
- `p240 -> R=60`
- `p300 -> R=0`

Total packets per frame: `N = K + R`.

## Simulator CLI (`meshcam_codec_sim.py`)

### Common options

- `--profile {p100,p200,p240,p300}`
- `--input <path>`
- `--loss-rate <0..1>`
- `--trials <int>`
- `--repair <int>`
- `--packet-limit <int>`
- `--output-dir <dir>`
- `--output-format {png,ppm}`
- `--conceal-missing` / `--no-conceal-missing`

### Save packet dumps

- `--save-packets`
- `--packet-dump-prefix <name>`

Example:

```bash
python meshcam_codec_sim.py \
  --profile p200 \
  --input ../aerial_sample.jpg \
  --loss-rate 0.25 \
  --trials 1 \
  --no-save-images \
  --save-packets \
  --output-dir out_packet_test \
  --packet-dump-prefix aerial
```

Generated files:

- `aerial_tx.bin`
- `aerial_tx.hex.txt`
- `aerial_rx.bin`
- `aerial_rx.hex.txt`
- `stats.txt`

### Output images

When image saving is enabled:

- `original.<ext>`
- `decoded_direct.<ext>`
- `decoded_fec.<ext>`
- `diff_fec_abs.<ext>`
- `stats.txt`

## Packet Dump Formats

### Binary dump (`*.bin`)

File structure:

1. Magic: `MCAMPKT1` (8 bytes)
2. Record size: 2-byte big-endian (`58`)
3. Record count: 4-byte big-endian
4. Records (58 bytes each):
   - Header (10 bytes)
   - Payload (48 bytes)

Per-record header layout:

- Byte `0`: flags (`bit0=1` repair, `0` data)
- Byte `1`: header version (`1`)
- Bytes `2..3`: `frame_id` (big-endian)
- Bytes `4..5`: `symbol_id` (big-endian)
- Bytes `6..7`: `k_data` (big-endian)
- Bytes `8..9`: `n_total` (big-endian)

### Hex dump (`*.hex.txt`)

- First line is metadata comment with magic/record size/count.
- One record per line:
  - packet index
  - symbol id
  - type (`D` or `R`)
  - full record bytes as hex

## Using the Codec as a Python Object

Instantiate `MeshCamCodec` directly (no singleton required).

Example:

```python
from meshcam_codec import HEIGHT, WIDTH, MeshCamCodec
from meshcam_codec_sim import apply_packet_loss, generate_synthetic_image

codec = MeshCamCodec()
profile = codec.get_profile("p200")
rgb = generate_synthetic_image(WIDTH, HEIGHT)

data_payloads = codec.encode_image_to_data_payloads(rgb, profile)
packets = codec.build_packets(data_payloads, frame_id=1, repair_count=100)

received, lost = apply_packet_loss(packets, loss_rate=0.25, seed=42)
recovered_payloads, solved, missing_before, rank = codec.recover_data_payloads(
    received,
    frame_id=1,
    k_data=profile.data_tiles,
)

decoded = codec.decode_image_from_data_payloads(
    recovered_payloads,
    profile,
    conceal_missing=True,
)
```

Useful object methods:

- `profile_names()`
- `get_profile(name)`
- `default_repair_count(profile)`
- `encode_image_to_data_payloads(rgb, profile)`
- `build_packets(data_payloads, frame_id, repair_count)`
- `recover_data_payloads(received, frame_id, k_data)`
- `decode_image_from_data_payloads(payloads, profile, conceal_missing=False)`
- `save_packet_dumps(base_path, packets)`

Simulation helpers in `meshcam_codec_sim.py`:

- `apply_packet_loss(packets, loss_rate, seed)`
- `mse_rgb(a, b)`
- `psnr_rgb(a, b)`
- `rgb_diff_stats(reference, test)`
- `build_abs_diff_image(reference, test, gain)`
- `generate_synthetic_image(width, height)`

## Notes

- The implementation currently supports only `320x240` geometry.
- At high loss rates, profiles with larger `K` may fail full FEC recovery if expected received packets are below `K`.
- `--conceal-missing` fills unrecovered tiles using nearest recovered content to avoid visible holes.
