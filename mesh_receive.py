#!/usr/bin/env python3
"""Receive MeshCam codec packets over Meshtastic and reconstruct an image."""

from __future__ import annotations

import argparse
import os
import queue
from sys import prefix
import time
from datetime import datetime
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

try:
    import meshtastic.ble_interface as ble_interface
    import meshtastic.serial_interface as serial_interface
    from meshtastic.protobuf import mesh_pb2, telemetry_pb2
except ImportError as exc:
    raise RuntimeError(
        "Meshtastic Python package is required. Install with: pip install meshtastic"
    ) from exc

from meshcam_codec import CODEC_PROFILES, MeshCamCodec, Packet

try:
    from pubsub import pub
except ImportError as exc:
    raise SystemExit("pypubsub is required. Install with: pip install pypubsub") from exc

P200_K_DATA = 200
P200_REPAIR = 100
PACKET_HEADER_SIZE = 4
PAYLOAD_IMAGE_BYTES = 48
PAYLOAD_METADATA_BYTES = 1
PAYLOAD_SIZE = PAYLOAD_IMAGE_BYTES + PAYLOAD_METADATA_BYTESPACKET_RECORD_SIZE = PACKET_HEADER_SIZE + PAYLOAD_SIZE

DEFAULT_OUTPUT_FORMAT = "png"
DEFAULT_CONCEAL_MISSING = True
IMAGE_PORT_NUM = 256 + 2

PROFILE_BY_K = {profile.data_tiles: profile for profile in CODEC_PROFILES.values()}


@dataclass
class FrameBuffer:
    sender: str
    frame_id: int
    k_data: int
    n_total: int
    packets_by_symbol: Dict[int, Packet] = field(default_factory=dict)
    first_seen: float = field(default_factory=datetime.now)
    last_seen: float = field(default_factory=time.monotonic)
    receivers: set[str] = field(default_factory=set)
    was_decoded: bool = False

    def add_symbol(self, packet: Packet) -> bool:
        if packet.k_data != self.k_data:
            return False
        if packet.symbol_id < 0:
            return False

        # Compact p200 wire packets do not carry total-N, so track an observed upper bound.
        self.n_total = max(self.n_total, packet.n_total, packet.symbol_id + 1)

        is_new = packet.symbol_id not in self.packets_by_symbol
        self.packets_by_symbol[packet.symbol_id] = packet
        if is_new:
            self.last_seen = time.monotonic()
            self.was_decoded = False
        return is_new

    def add_receiver(self, receiver_id: str) -> None:
        self.receivers.add(receiver_id)

    def received_count(self) -> int:
        return len(self.packets_by_symbol)

    def packets(self) -> list[Packet]:
        return [self.packets_by_symbol[sid] for sid in sorted(self.packets_by_symbol)]


def open_interface(serial_arg: str | None, ble_address: str | None) -> Any:
        if serial_arg is not None:
            serial_port = serial_arg if serial_arg else None
            return serial_interface.SerialInterface(devPath=serial_port)
        else:
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


def parse_codec_record(record: bytes) -> Packet:
    if len(record) != PACKET_RECORD_SIZE:
        raise ValueError("Unexpected packet record length")

    frame_id = int.from_bytes(record[0:2], "big")
    symbol_id = int.from_bytes(record[2:4], "big")
    payload = bytes(record[4:])

    if len(payload) != PAYLOAD_SIZE:
        raise ValueError("Unexpected payload length")

    return Packet(
        frame_id=frame_id,
        symbol_id=symbol_id,
        k_data=P200_K_DATA,
        n_total=P200_K_DATA + P200_REPAIR,
        is_repair=(symbol_id >= P200_K_DATA),
        payload=payload,
    )


def recover_metadata(packets: list[Packet]) -> str:
    data_by_sid = {p.symbol_id: p for p in packets if not p.is_repair}
    chars = []
    for sid in range(max(data_by_sid, default=-1) + 1):
        p = data_by_sid.get(sid)
        if p is None or p.payload[-1] == 0:
            break
        chars.append(chr(p.payload[-1]))
    return "".join(chars)


def try_decode_frame(
    codec: MeshCamCodec,
    frame: FrameBuffer,
    output_dir: str,
) -> bool:
    try:
        profile = PROFILE_BY_K.get(frame.k_data)
        if profile is None:
            raise RuntimeError(f"No codec profile matches k_data={frame.k_data}")

        received_packets = frame.packets()
        recovered_payloads, solved, missing_before, rank = codec.recover_data_payloads(
            received_packets,
            frame_id=frame.frame_id,
            k_data=frame.k_data,
        )
        final_missing = sum(1 for payload in recovered_payloads if payload is None)
        full_recovery = bool(solved and final_missing == 0)

        rgb = codec.decode_image_from_data_payloads(
            recovered_payloads,
            profile,
            conceal_missing=DEFAULT_CONCEAL_MISSING,
        )
    except RuntimeError as exc:
        print(f"Cannot decode frame {frame.frame_id}: {exc}")
        return False
    
    metadata = recover_metadata(frame.packets())
    timestamp = frame.first_seen.strftime("%Y%m%d_%H%M%S")
    image_path = os.path.join(output_dir, f"meshcam_{timestamp}_{frame.frame_id:04x}")
    try:
        codec.save_image(image_path+".png", rgb)
    except Exception as exc:
        print(f"Failed to save image for frame {frame.frame_id}: {exc}")
    try:
        with open(image_path+".stats.txt", "w", encoding="ascii") as f:
            f.write("MeshCam Meshtastic Receive\n")
            f.write(f"Sender={frame.sender}\n")
            f.write(f"FrameId={frame.frame_id:04x}\n")
            f.write(f"Profile={profile.name}\n")
            f.write(f"K={frame.k_data}\n")
            f.write(f"N={frame.n_total}\n")
            f.write(f"ReceivedSymbols={frame.received_count()}\n")
            f.write(f"MissingBeforeFEC={missing_before}\n")
            f.write(f"SolveRank={rank}\n")
            f.write(f"MissingAfterFEC={final_missing}\n")
            f.write(f"FullRecovery={full_recovery}\n")
            f.write(f"ConcealMissing={DEFAULT_CONCEAL_MISSING}\n")
            f.write(f"Timestamp={timestamp}\n")
            f.write(f"Metadata={metadata}\n")

        print(
            "Saved frame "
            f"{frame.frame_id} -> {image_path} "
            f"(received={frame.received_count()}/{frame.n_total}, "
            f"missing_before={missing_before}, rank={rank}, missing_after={final_missing}, "
            f"full_recovery={full_recovery})"
            f"Metadata: {metadata!r}"
            )
    except Exception as exc:
        print(f"Failed to save stats for frame {frame.frame_id}: {exc}")

    try:
        codec.save_packet_dumps(image_path+".packets.txt", frame.packets())
    except Exception as exc:
        print(f"Failed to save packet dumps for frame {frame.frame_id}: {exc}")

    frame.was_decoded = True
    return full_recovery

def build_nodes_db(interface: Any) -> None:
    nodes = interface.nodes

    if nodes:
        for node_id, node_info in nodes.items():
            user = node_info.get('user', {})
            print(f"Node ID: {node_id}")
            print(f"  Long Name: {user.get('longName', 'N/A')}")
            print(f"  Short Name: {user.get('shortName', 'N/A')}")
            print(f"  Hardware: {user.get('hwModel', 'Unknown')}")
    else:
        print("No nodes found in NodeDB.")

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive MeshCam image packets from Meshtastic")
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
        "--idle-timeout",
        type=float,
        default=10.0,
        help="Decode best frame after this many seconds without MeshCam packets",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=180.0,
        help="delete frame after this many seconds without any packets (prevents memory leak from incomplete frames)",
    )

    parser.add_argument("--track", type=str, default="", help="track info from specific node")
    parser.add_argument("--channel", type=int, default=0, help="channel index")
    parser.add_argument("--output-dir", type=str, default="received", help="Output directory")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.ble == "":
        raise SystemExit(list_ble_devices())

    os.makedirs(args.output_dir, exist_ok=True)

    codec = MeshCamCodec()
    frames: Dict[int, FrameBuffer] = {}
    saved_frame_ids: set[int] = set()
    inbox: queue.Queue[Dict[str, Any]] = queue.Queue()

    def on_receive(packet: Dict[str, Any], interface: Any) -> None:
        del interface
        inbox.put(packet)

    print('=' * 40)
    print("MeshCam receiver")
    transport = "serial" if args.serial is not None else "ble"
    print(f"connecting to node using {transport}...")

    pub.subscribe(on_receive, "meshtastic.receive")
    interface = open_interface(args.serial, args.ble)

    node_num = interface.myInfo.my_node_num
    print(f"Node ID: !{node_num:08x}")

    print(f"Output dir: {args.output_dir}")
    print('=' * 40)

    try:
        while True:
            try:
                packet = inbox.get(timeout=0.5)
                channel = packet.get("channel", 0)
                from_node = packet.get("fromId", "?")
                decoded = packet.get("decoded", {})
                payload = decoded.get("payload")
                portnum = decoded.get("portnum")
                
                if channel is not None and channel != args.channel:
                    continue

                header = f"[{str(portnum).replace('_APP', '')} {from_node} ch.{channel}] "

                if portnum == "POSITION_APP":
                    pos = mesh_pb2.Position()
                    pos.ParseFromString(payload)
                    lat = pos.latitude_i / 1e7 if pos.latitude_i else None
                    lon = pos.longitude_i / 1e7 if pos.longitude_i else None
                    alt = pos.altitude if pos.altitude else None
                    speed = pos.ground_speed or 0
                    if args.verbose or from_node == args.track:
                        print(f"{header} lat={lat} lon={lon} alt={alt}m speed={speed}m/s ")
                elif portnum == "TELEMETRY_APP":
                    tel = telemetry_pb2.Telemetry()
                    tel.ParseFromString(payload)
                    m = tel.device_metrics
                    if args.verbose or from_node == args.track:
                        print(f"{header} battery={m.battery_level}% voltage={m.voltage:.2f}V uptime={m.uptime_seconds/3600:.2f}h")
                elif portnum == "NODEINFO_APP":
                    user = mesh_pb2.User()
                    user.ParseFromString(payload)
                    if args.verbose or from_node == args.track:
                        print(f"{header} {user.long_name!r} ({user.short_name}) id={user.id} hw={user.hw_model}")
                elif portnum == IMAGE_PORT_NUM:
                    symbol = parse_codec_record(payload)
                    if symbol.frame_id in saved_frame_ids:
                        print(header, f"Ignoring packet for already saved frame {symbol.frame_id} from node {from_node}")
                        continue

                    frame = frames.get(symbol.frame_id)
                    if frame is None:
                        frame = FrameBuffer(from_node, symbol.frame_id, symbol.k_data, symbol.n_total)
                        frame.add_receiver(node_num)
                        frames[symbol.frame_id] = frame
                        print(header, f"New frame {symbol.frame_id:04X} "
                            f"(K={symbol.k_data}, N={symbol.n_total})"
                        )

                    is_new = frame.add_symbol(symbol)
                    if not is_new:
                        print(header, f"Ignoring duplicate packet symbol {symbol.symbol_id} for frame {symbol.frame_id} from node {from_node}")
                        continue

                    count = frame.received_count()
                    print(header, f"Frame {frame.frame_id:04X}: received {len(payload)}B {count}/{frame.n_total} symbol id = {symbol.symbol_id} ")

                else:
                    print(header, f"Received packet of {len(payload) if payload else '?'} bytes")

            except queue.Empty:
                for frame_id, frame in list(frames.items()):
                    if (frame.received_count() > frame.k_data) or (args.idle_timeout > 0 and (time.monotonic() - frame.last_seen) >= args.idle_timeout):
                        if not frame.was_decoded:
                            full_recovery = try_decode_frame(
                                codec,
                                frame,
                                args.output_dir
                            )
                            print(f"[INFO] Frame {frame_id:04X} decode result: full_recovery={full_recovery} received={frame.received_count()}/{frame.n_total}")
                            if full_recovery:
                                saved_frame_ids.add(frame_id)
                                frames.pop(frame_id, None)
               
                if args.stale_timeout > 0 and frames and (time.monotonic() - frame.last_seen) >= args.stale_timeout:
                    print(f"[INFO] Stale timeout reached ({time.monotonic() - frame.last_seen:.1f}s>{args.stale_timeout}s); ")                        
                    saved_frame_ids.add(frame.frame_id)
                    frames.pop(frame.frame_id, None)
            except Exception as exc:
                print(f"[ERROR] Error processing packet: {exc}")

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user")
    finally:
        interface.close()
        pub.unsubscribe(on_receive, "meshtastic.receive")


if __name__ == "__main__":
    main()
