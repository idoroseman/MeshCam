#!/usr/bin/env python3
"""Send MeshCam codec packets over a Meshtastic network.

Each MeshCam codec symbol is sent as one Meshtastic data payload:
- Compact p200 wire payload: frame_id(2) + symbol_id(2) + payload(48)
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path
from typing import Any

from meshcam_codec import MeshCamCodec, Packet

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

try:
    import meshtastic.serial_interface as serial_interface
    import meshtastic.ble_interface as ble_interface
except ImportError as exc:
    raise RuntimeError(
        "Meshtastic Python package is required. Install with: pip install meshtastic"
    ) from exc

P200_PROFILE_NAME = "p200"
P200_K_DATA = 200
PACKET_HEADER_SIZE = 4
PAYLOAD_IMAGE_BYTES = 48
PAYLOAD_METADATA_BYTES = 1
PAYLOAD_SIZE = PAYLOAD_IMAGE_BYTES + PAYLOAD_METADATA_BYTES
PACKET_RECORD_SIZE = PACKET_HEADER_SIZE + PAYLOAD_SIZE
IMAGE_PORT_NUM = 256 + 2
DEFAULT_DESTINATION_ID = "^all"



def build_wire_payload(packet: Packet) -> bytes:
    if packet.k_data != P200_K_DATA:
        raise ValueError("Compact wire format supports only p200 (k_data=200)")
    if packet.frame_id < 0 or packet.frame_id > 0xFFFF:
        raise ValueError("frame_id must be in [0, 65535]")
    if packet.symbol_id < 0 or packet.symbol_id > 0xFFFF:
        raise ValueError("symbol_id must be in [0, 65535]")
    if len(packet.payload) != PAYLOAD_SIZE:
        raise ValueError("Packet payload has an unexpected size")

    return bytes(
        [
            (packet.frame_id >> 8) & 0xFF,
            packet.frame_id & 0xFF,
            (packet.symbol_id >> 8) & 0xFF,
            packet.symbol_id & 0xFF,
        ]
    ) + packet.payload


def open_interface(serial_arg: str | None, ble_address: str | None) -> Any:
    if serial_arg is not None:
        serial_port = serial_arg if serial_arg else None
        return serial_interface.SerialInterface(devPath=serial_port)

    return ble_interface.BLEInterface(address=ble_address)



def list_ble_devices() -> int:
    try:
        import meshtastic.ble_interface as ble_interface
    except ImportError:
        print("Meshtastic Python package is required. Install with: pip install meshtastic")
        return 1

    try:
        devices = ble_interface.BLEInterface.scan()
    except Exception as exc:
        print(f"BLE scan failed: {exc}")
        return 1

    if not devices:
        print("No BLE devices found.")
        return 0

    print("Available BLE devices:")
    for idx, device in enumerate(devices, start=1):
        address = getattr(device, "address", "unknown")
        name = getattr(device, "name", None) or "(no name)"
        print(f"{idx:2d}. {address}  {name}")
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Encode and send one MeshCam image over Meshtastic")
    parser.add_argument("--image", "--input", dest="image", type=Path, required=False, help="Input image path")
    parser.add_argument(
        "--frame-id",
        type=int,
        default=(int(time.time()) & 0xFFFF),
        help="Frame id written into packet headers (0..65535)",
    )
    transport_group = parser.add_mutually_exclusive_group(required=True)
    transport_group.add_argument(
        "--serial",
        nargs="?",
        const="",
        default=None,
        metavar="PORT",
        help="Use serial transport. Optionally provide a serial device path.",
    )
    transport_group.add_argument(
        "--ble",
        nargs="?",
        const="",
        default=None,
        metavar="ADDRESS",
        help="Use BLE transport. If ADDRESS is omitted, list available BLE devices.",
    )
    parser.add_argument(
        "--port-num",
        type=int,
        default=IMAGE_PORT_NUM,
        help="Meshtastic application port number",
    )
    parser.add_argument("--channel-index", type=int, default=0, help="Meshtastic channel index")
    parser.add_argument("--want-ack", action="store_true", help="Request transport ACK for each packet")
    parser.add_argument(
        "--inter-packet-delay",
        type=float,
        default=1.5,
        help="Delay in seconds between packets",
    )
    parser.add_argument(
        "--metadata",
        type=str,
        default="",
        help=f"Metadata string (up to {P200_K_DATA} chars); n-th char embedded in n-th data packet payload",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.ble == "":
        raise SystemExit(list_ble_devices())
    if args.image is None:
        raise ValueError("--image is required when sending an image")

    if args.frame_id < 0 or args.frame_id > 0xFFFF:
        raise ValueError("--frame-id must be in [0, 65535]")
    if args.inter_packet_delay < 0:
        raise ValueError("--inter-packet-delay must be >= 0")

    codec = MeshCamCodec()
    profile = codec.get_profile(P200_PROFILE_NAME)
    repair_count = profile.repair_tiles

    print('=' * 40)
    print("MeshCam sender")

    transport = "serial" if args.serial is not None else "ble"
    print(f"connecting to node using {transport}...")
    interface = open_interface(args.serial, args.ble)
    node_num = interface.myInfo.my_node_num
    local_node = (interface.nodes or {}).get(f"!{node_num:08x}", {})
    user = local_node.get("user", {})
    print(f"Node ID: !{node_num:08x}  {user.get('longName', '')} ({user.get('shortName', '')})")
    position = local_node.get("position", {})
    lat_i = position.get("latitudeI", 0)
    lon_i = position.get("longitudeI", 0)
    if lat_i and lon_i:
        alt = position.get("altitude")
        alt_str = f" alt={alt}m" if alt else ""
        print(f"GPS: {lat_i / 1e7:.6f}, {lon_i / 1e7:.6f}{alt_str}")
    else:
        print("GPS: no fix")
        
    print(f"Input: {args.image}")
    print(f"Profile: {profile.name}")
    print(f"Frame ID: {args.frame_id:04X}")

    annotation_lines =  []

    long_name = user.get("longName", "")
    short_name = user.get("shortName", "")
    if long_name:
        annotation_lines.append(long_name)
    if short_name:
        annotation_lines.append(f"({short_name})")
    if lat_i and lon_i:
        lat, lon = lat_i / 1e7, lon_i / 1e7
        annotation_lines.append(f"{lat:.5f}, {lon:.5f}")
        if alt:
            annotation_lines.append(f"alt {alt}m")
    if args.metadata:
        annotation_lines.append(args.metadata)
        
    rgb = codec.load_image_rgb(args.image)
    data_payloads = codec.encode_image_to_data_payloads(rgb, profile)
    packets = codec.build_packets(data_payloads, frame_id=args.frame_id, repair_count=repair_count, metadata=" ".join(annotation_lines))

    print(f"Data packets (K): {profile.data_tiles}")
    print(f"Repair packets (R): {repair_count}")
    print(f"Packets to send: {len(packets)}")
    print(f"Wire record size: {PACKET_RECORD_SIZE} bytes")
    print(f"Destination: {DEFAULT_DESTINATION_ID}")
    print(f"Port number: {args.port_num}")
    print(f"Channel index: {args.channel_index}")

    print('=' * 40)

    tick = time.time()
    try:
        for idx, packet in enumerate(packets, start=1):
            wire_payload = build_wire_payload(packet)
            interface.sendData(
                wire_payload,
                destinationId=DEFAULT_DESTINATION_ID,
                portNum=args.port_num,
                wantAck=args.want_ack,
                wantResponse=False,
                channelIndex=args.channel_index,
            )

            packet_type = "R" if packet.is_repair else "D"
            print(f"Sent {idx}/{len(packets)} symbol={packet.symbol_id} type={packet_type}", end="\r")

            if args.inter_packet_delay > 0 and idx < len(packets):
                time.sleep(args.inter_packet_delay)

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        toc = time.time()
        elapsed = toc - tick
        print(f"\nElapsed time: {elapsed/60:.2f} minutes")
        print()  # newline after progress
        print("Closing interface...")
        t = threading.Thread(target=interface.close, daemon=True)
        t.start()
        t.join(timeout=5)
        if t.is_alive():
            print("Warning: interface.close() timed out, forcing exit")
    print("Done sending")


if __name__ == "__main__":
    main()
