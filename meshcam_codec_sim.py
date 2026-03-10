#!/usr/bin/env python3
"""MeshCam codec simulation CLI.

This file is intentionally a thin runner around `MeshCamCodec`.
Core codec/FEC/packet logic lives in `meshcam_codec.py`.
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path
from typing import Dict, List, Optional

from meshcam_codec import (
    CODEC_PROFILES,
    HEIGHT,
    WIDTH,
    CodecProfile,
    MeshCamCodec,
    Packet,
)


def apply_packet_loss(packets: List[Packet], loss_rate: float, seed: int) -> tuple[List[Packet], int]:
    rng = random.Random(seed)
    received: List[Packet] = []
    lost = 0
    for pkt in packets:
        if rng.random() < loss_rate:
            lost += 1
        else:
            received.append(pkt)
    return received, lost


def mse_rgb(a: bytes, b: bytes) -> float:
    n = len(a)
    if n != len(b) or n == 0:
        raise ValueError("RGB buffers must have equal non-zero length")
    total = 0
    for i in range(n):
        d = int(a[i]) - int(b[i])
        total += d * d
    return total / n


def psnr_rgb(a: bytes, b: bytes) -> float:
    err = mse_rgb(a, b)
    if err == 0:
        return float("inf")
    return 10.0 * math.log10((255.0 * 255.0) / err)


def build_abs_diff_image(reference: bytes, test: bytes, gain: int) -> bytes:
    if len(reference) != len(test):
        raise ValueError("RGB buffers must have equal length")
    if gain < 1:
        gain = 1

    out = bytearray(len(reference))
    for i in range(len(reference)):
        err = abs(int(test[i]) - int(reference[i])) * gain
        out[i] = 255 if err > 255 else err
    return bytes(out)


def rgb_diff_stats(reference: bytes, test: bytes) -> Dict[str, float]:
    if len(reference) != len(test) or (len(reference) % 3) != 0:
        raise ValueError("RGB buffers must have equal length and 3-byte pixels")

    pixels = len(reference) // 3
    sum_abs = 0
    sum_sq = 0
    sum_abs_r = 0
    sum_abs_g = 0
    sum_abs_b = 0
    sum_sq_r = 0
    sum_sq_g = 0
    sum_sq_b = 0
    max_abs_err = 0
    identical_pixels = 0

    for p in range(pixels):
        i = 3 * p
        dr = abs(int(test[i + 0]) - int(reference[i + 0]))
        dg = abs(int(test[i + 1]) - int(reference[i + 1]))
        db = abs(int(test[i + 2]) - int(reference[i + 2]))

        if dr == 0 and dg == 0 and db == 0:
            identical_pixels += 1

        sum_abs_r += dr
        sum_abs_g += dg
        sum_abs_b += db
        sum_abs += dr + dg + db

        sum_sq_r += dr * dr
        sum_sq_g += dg * dg
        sum_sq_b += db * db
        sum_sq += (dr * dr) + (dg * dg) + (db * db)

        if dr > max_abs_err:
            max_abs_err = dr
        if dg > max_abs_err:
            max_abs_err = dg
        if db > max_abs_err:
            max_abs_err = db

    mse = sum_sq / (pixels * 3)

    return {
        "pixels": float(pixels),
        "mae": sum_abs / (pixels * 3),
        "mse": mse,
        "psnr": psnr_rgb(reference, test),
        "mae_r": sum_abs_r / pixels,
        "mae_g": sum_abs_g / pixels,
        "mae_b": sum_abs_b / pixels,
        "mse_r": sum_sq_r / pixels,
        "mse_g": sum_sq_g / pixels,
        "mse_b": sum_sq_b / pixels,
        "max_abs_err": float(max_abs_err),
        "identical_pixels": float(identical_pixels),
        "identical_pct": (100.0 * identical_pixels) / pixels,
    }


def generate_synthetic_image(width: int, height: int) -> bytes:
    rgb = bytearray(width * height * 3)
    for y in range(height):
        for x in range(width):
            i = y * width + x
            r = (x * 255) // (width - 1)
            g = (y * 255) // (height - 1)
            b = ((x ^ y) * 255) // 511
            rgb[3 * i + 0] = r
            rgb[3 * i + 1] = g
            rgb[3 * i + 2] = b
    return bytes(rgb)

def run_trial(
    codec: MeshCamCodec,
    packets: List[Packet],
    original_rgb: bytes,
    frame_id: int,
    k_data: int,
    loss_rate: float,
    seed: int,
    profile: CodecProfile,
    conceal_missing: bool,
) -> Dict[str, object]:
    received_packets, lost = apply_packet_loss(packets, loss_rate=loss_rate, seed=seed)

    direct_payloads: List[Optional[bytes]] = [None] * k_data
    for pkt in received_packets:
        if pkt.symbol_id < k_data:
            direct_payloads[pkt.symbol_id] = pkt.payload

    direct_missing = sum(1 for payload in direct_payloads if payload is None)
    direct_rgb = codec.decode_image_from_data_payloads(direct_payloads, profile, conceal_missing=False)
    direct_psnr = psnr_rgb(original_rgb, direct_rgb)

    recovered_payloads, solved, missing_before, rank = codec.recover_data_payloads(
        received_packets,
        frame_id=frame_id,
        k_data=k_data,
    )
    final_missing = sum(1 for payload in recovered_payloads if payload is None)
    recovered = bool(solved and final_missing == 0)
    final_rgb = codec.decode_image_from_data_payloads(
        recovered_payloads,
        profile,
        conceal_missing=conceal_missing,
    )
    final_psnr = psnr_rgb(original_rgb, final_rgb)

    return {
        "received": len(received_packets),
        "lost": lost,
        "direct_missing": direct_missing,
        "fec_missing_before": missing_before,
        "fec_rank": rank,
        "recovered": recovered,
        "final_missing": final_missing,
        "direct_psnr": direct_psnr,
        "final_psnr": final_psnr,
        "direct_rgb": direct_rgb,
        "final_rgb": final_rgb,
        "received_packets": list(received_packets),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="MeshCam color tile codec + packet-loss simulator")
    parser.add_argument(
        "--profile",
        type=str,
        default="p200",
        choices=sorted(CODEC_PROFILES.keys()),
        help="Codec profile: p100=100 data, p200=200 data, p240=240 data, p300=300 data packets",
    )
    parser.add_argument("--input", type=Path, default=None, help="Input image path (optional)")
    parser.add_argument("--output-dir", type=Path, default=Path("out"), help="Directory for outputs")
    parser.add_argument("--frame-id", type=int, default=1, help="Frame id for packet headers")
    parser.add_argument(
        "--repair",
        type=int,
        default=-1,
        help="Number of repair packets; use -1 to pick profile default",
    )
    parser.add_argument(
        "--packet-limit",
        type=int,
        default=300,
        help="Maximum allowed total packets (data + repair)",
    )
    parser.add_argument("--loss-rate", type=float, default=0.25, help="Packet drop probability (0..1)")
    parser.add_argument("--seed", type=int, default=12345, help="Random seed for channel simulation")
    parser.add_argument("--trials", type=int, default=1, help="Number of channel trials")
    parser.add_argument(
        "--output-format",
        type=str,
        default="png",
        choices=["png", "ppm"],
        help="Image output format",
    )
    parser.add_argument(
        "--no-save-images",
        action="store_true",
        help="Do not write output image files",
    )
    parser.add_argument(
        "--save-packets",
        action="store_true",
        help="Save transmitted and received packets as binary and hex dump files",
    )
    parser.add_argument(
        "--packet-dump-prefix",
        type=str,
        default="packets",
        help="Base filename prefix for saved packet dumps",
    )
    parser.add_argument(
        "--diff-gain",
        type=int,
        default=6,
        help="Brightness multiplier for saved absolute-difference image",
    )
    parser.add_argument(
        "--conceal-missing",
        dest="conceal_missing",
        action="store_true",
        help="Conceal unrecovered tiles in the final decoded image",
    )
    parser.add_argument(
        "--no-conceal-missing",
        dest="conceal_missing",
        action="store_false",
        help="Leave unrecovered tiles as neutral blocks in the final decoded image",
    )
    parser.set_defaults(conceal_missing=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    codec = MeshCamCodec()
    profile = codec.get_profile(args.profile)
    k_data = profile.data_tiles

    repair = args.repair
    if repair < 0:
        repair = profile.repair_tiles

    if not (0.0 <= args.loss_rate <= 1.0):
        raise ValueError("--loss-rate must be in [0, 1]")
    if repair < 0:
        raise ValueError("--repair must be >= 0")
    if args.trials < 1:
        raise ValueError("--trials must be >= 1")
    if args.packet_limit < k_data:
        raise ValueError("--packet-limit is below the data packet count for the selected profile")

    n_total = k_data + repair
    if n_total > args.packet_limit:
        raise ValueError("Total packets exceed --packet-limit. Lower --repair or raise the limit.")

    if args.input is not None:
        original_rgb = codec.load_image_rgb(args.input)
        input_desc = str(args.input)
    else:
        original_rgb = generate_synthetic_image(WIDTH, HEIGHT)
        input_desc = "synthetic"

    data_payloads = codec.encode_image_to_data_payloads(original_rgb, profile)
    packets = codec.build_packets(data_payloads, args.frame_id, repair)

    first = run_trial(
        codec=codec,
        packets=packets,
        original_rgb=original_rgb,
        frame_id=args.frame_id,
        k_data=k_data,
        loss_rate=args.loss_rate,
        seed=args.seed,
        profile=profile,
        conceal_missing=args.conceal_missing,
    )
    fec_diff = rgb_diff_stats(original_rgb, first["final_rgb"])

    print("MeshCam Codec Simulation")
    print(f"Input: {input_desc}")
    print(f"Image: {WIDTH}x{HEIGHT}")
    print(f"Profile: {profile.name}")
    print(f"Data packets (K): {k_data}")
    print(f"Repair packets (R): {repair}")
    print(f"Total packets (N): {n_total}")
    print(f"Packet limit: {args.packet_limit}")
    print(f"Loss rate: {args.loss_rate:.2%}")
    print(f"Conceal missing tiles: {args.conceal_missing}")
    expected_received = n_total * (1.0 - args.loss_rate)
    if expected_received < k_data:
        print(
            "Warning: expected received packets "
            f"({expected_received:.1f}) are below K ({k_data}); "
            "full FEC recovery is unlikely at this loss rate."
        )
    print("-")
    print(f"Trial seed: {args.seed}")
    print(f"Received packets: {first['received']} / {n_total}")
    print(f"Lost packets: {first['lost']}")
    print(f"Missing data tiles before FEC: {first['fec_missing_before']}")
    print(f"FEC solve rank: {first['fec_rank']}")
    print(f"Full recovery: {first['recovered']}")
    print(f"Missing data tiles after FEC: {first['final_missing']}")
    print(f"PSNR direct-only: {first['direct_psnr']:.2f} dB")
    print(f"PSNR after FEC: {first['final_psnr']:.2f} dB")
    print(f"MAE after FEC: {fec_diff['mae']:.2f}")
    print(f"Max absolute error after FEC: {int(fec_diff['max_abs_err'])}")
    print(
        "Identical pixels after FEC: "
        f"{int(fec_diff['identical_pixels'])}/{int(fec_diff['pixels'])} "
        f"({fec_diff['identical_pct']:.2f}%)"
    )

    if args.trials > 1:
        recovered_count = 1 if first["recovered"] else 0
        direct_psnr_sum = float(first["direct_psnr"])
        final_psnr_sum = float(first["final_psnr"])

        for t in range(1, args.trials):
            seed = args.seed + (t * 7919)
            result = run_trial(
                codec=codec,
                packets=packets,
                original_rgb=original_rgb,
                frame_id=args.frame_id,
                k_data=k_data,
                loss_rate=args.loss_rate,
                seed=seed,
                profile=profile,
                conceal_missing=args.conceal_missing,
            )
            if result["recovered"]:
                recovered_count += 1
            direct_psnr_sum += float(result["direct_psnr"])
            final_psnr_sum += float(result["final_psnr"])

        print("-")
        print(f"Trials: {args.trials}")
        print(f"Recovery success rate: {100.0 * recovered_count / args.trials:.2f}%")
        print(f"Average PSNR direct-only: {direct_psnr_sum / args.trials:.2f} dB")
        print(f"Average PSNR after FEC: {final_psnr_sum / args.trials:.2f} dB")

    write_outputs = (not args.no_save_images) or args.save_packets
    if write_outputs:
        args.output_dir.mkdir(parents=True, exist_ok=True)

    if not args.no_save_images:
        suffix = ".png" if args.output_format == "png" else ".ppm"

        codec.save_image(args.output_dir / f"original{suffix}", original_rgb)
        codec.save_image(args.output_dir / f"decoded_direct{suffix}", first["direct_rgb"])
        codec.save_image(args.output_dir / f"decoded_fec{suffix}", first["final_rgb"])
        diff_image = build_abs_diff_image(original_rgb, first["final_rgb"], args.diff_gain)
        codec.save_image(args.output_dir / f"diff_fec_abs{suffix}", diff_image)

    tx_bin_path: Optional[Path] = None
    tx_hex_path: Optional[Path] = None
    rx_bin_path: Optional[Path] = None
    rx_hex_path: Optional[Path] = None

    if args.save_packets:
        received_packets_obj = first["received_packets"]
        if not isinstance(received_packets_obj, list) or any(not isinstance(p, Packet) for p in received_packets_obj):
            raise RuntimeError("received packet dump is not available")

        tx_bin_path, tx_hex_path = codec.save_packet_dumps(
            args.output_dir / f"{args.packet_dump_prefix}_tx",
            packets,
        )
        rx_bin_path, rx_hex_path = codec.save_packet_dumps(
            args.output_dir / f"{args.packet_dump_prefix}_rx",
            received_packets_obj,
        )

    if write_outputs:
        stats_path = args.output_dir / "stats.txt"
        with stats_path.open("w", encoding="ascii") as f:
            f.write("MeshCam Codec Simulation\n")
            f.write(f"Input: {input_desc}\n")
            f.write(f"Image: {WIDTH}x{HEIGHT}\n")
            f.write(f"Profile={profile.name}\n")
            f.write(f"K={k_data}, R={repair}, N={n_total}\n")
            f.write(f"ConcealMissing={args.conceal_missing}\n")
            f.write(f"Loss rate={args.loss_rate:.6f}\n")
            f.write(f"Seed={args.seed}\n")
            f.write(f"Received={first['received']}\n")
            f.write(f"Lost={first['lost']}\n")
            f.write(f"MissingBeforeFEC={first['fec_missing_before']}\n")
            f.write(f"SolveRank={first['fec_rank']}\n")
            f.write(f"Recovered={first['recovered']}\n")
            f.write(f"MissingAfterFEC={first['final_missing']}\n")
            f.write(f"PSNR_Direct={first['direct_psnr']:.4f}\n")
            f.write(f"PSNR_FEC={first['final_psnr']:.4f}\n")
            f.write(f"MAE_FEC={fec_diff['mae']:.4f}\n")
            f.write(f"MAX_ABS_ERR_FEC={int(fec_diff['max_abs_err'])}\n")
            f.write(f"IDENTICAL_PIXELS_FEC={int(fec_diff['identical_pixels'])}\n")
            f.write(f"IDENTICAL_PCT_FEC={fec_diff['identical_pct']:.4f}\n")
            f.write(f"SavePackets={args.save_packets}\n")
            if tx_bin_path is not None and tx_hex_path is not None:
                f.write(f"PacketDumpTXBin={tx_bin_path.name}\n")
                f.write(f"PacketDumpTXHex={tx_hex_path.name}\n")
            if rx_bin_path is not None and rx_hex_path is not None:
                f.write(f"PacketDumpRXBin={rx_bin_path.name}\n")
                f.write(f"PacketDumpRXHex={rx_hex_path.name}\n")

        print(f"Saved outputs in: {args.output_dir}")
        if args.save_packets:
            print(f"Saved TX packet dumps: {tx_bin_path.name}, {tx_hex_path.name}")
            print(f"Saved RX packet dumps: {rx_bin_path.name}, {rx_hex_path.name}")


if __name__ == "__main__":
    main()
