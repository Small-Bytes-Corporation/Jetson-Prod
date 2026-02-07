#!/usr/bin/env python3
"""
Script pour recevoir et afficher l'image de la caméra diffusée via UDP.

Le serveur (camera_stream.py ou main.py --enable-socket) envoie les frames
en paquets UDP chunkés (header !IHH + payload JPEG ou H264).

Usage: python scripts/camera_receiver.py [--port 3000]
       Ctrl+C pour quitter
"""

import argparse
import socket
import struct
import sys
import time
from datetime import datetime

import cv2
import numpy as np

try:
    import av
    HAS_AV = True
except ImportError:
    HAS_AV = False

# Configuration
CHUNK_HEADER_FORMAT = "!IHH"
CHUNK_HEADER_SIZE = struct.calcsize(CHUNK_HEADER_FORMAT)
UDP_PORT_DEFAULT = 3000


def main():
    parser = argparse.ArgumentParser(
        description="Recevoir et afficher l'image de la caméra diffusée via UDP"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=UDP_PORT_DEFAULT,
        help=f"Port UDP à écouter (défaut: {UDP_PORT_DEFAULT})",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Afficher les paquets reçus (debug)",
    )
    parser.add_argument(
        "--output",
        "-o",
        type=str,
        default=None,
        metavar="FILE",
        help="Fichier vidéo de sortie (défaut: recording_YYYYMMDD_HHMMSS.mp4)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=10.0,
        help="FPS pour la vidéo enregistrée (défaut: 10)",
    )
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", args.port))
    except OSError as e:
        print(f"[CameraReceiver] Erreur: impossible de se lier au port {args.port}: {e}")
        return 1

    sock.settimeout(0.1)
    frame_buffers = {}  # frame_number -> {total_chunks, chunks: {}}
    last_frame_time = None
    frame_count = 0
    packet_count = 0
    no_packet_warned = False
    h264_decoder = None
    video_writer = None
    output_path = None
    if HAS_AV:
        h264_decoder = av.CodecContext.create("h264", "r")

    print(f"[CameraReceiver] Écoute sur le port UDP {args.port}...")
    if HAS_AV:
        print("[CameraReceiver] Décodage JPEG + H264 (PyAV) supporté")
    else:
        print("[CameraReceiver] Décodage JPEG uniquement (pip install av pour H264)")
    print("[CameraReceiver] En attente de flux caméra... (Ctrl+C pour quitter)")
    if args.debug:
        print("[CameraReceiver] [debug] Mode debug activé")
    start_time = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                if args.debug and not no_packet_warned and (time.time() - start_time) > 2.0:
                    print("[CameraReceiver] [debug] Aucun paquet reçu - camera_stream tourne ? même port ?")
                    no_packet_warned = True
                continue
            except Exception as e:
                print(f"[CameraReceiver] Erreur recvfrom: {e}")
                continue

            packet_count += 1
            if args.debug and packet_count <= 50:
                print(f"[CameraReceiver] [debug] Paquet #{packet_count}: {len(data)} bytes, 1er octet=0x{data[0]:02x}, addr={addr}")

            if len(data) < CHUNK_HEADER_SIZE:
                if args.debug and packet_count <= 20:
                    print(f"[CameraReceiver] [debug] Paquet trop court ({len(data)} < {CHUNK_HEADER_SIZE}), ignoré")
                continue

            # Paquet chunké caméra (binaire)
            if data[0] != 0x7B:  # '{' = début JSON, on ignore les messages status
                try:
                    frame_num, total_chunks, chunk_idx = struct.unpack(
                        CHUNK_HEADER_FORMAT, data[:CHUNK_HEADER_SIZE]
                    )
                except struct.error as e:
                    if args.debug and packet_count <= 20:
                        print(f"[CameraReceiver] [debug] struct.unpack erreur: {e}")
                    continue
                payload = data[CHUNK_HEADER_SIZE:]
                if args.debug and packet_count <= 50:
                    print(f"[CameraReceiver] [debug] Chunk: frame={frame_num}, chunk={chunk_idx}/{total_chunks}, payload={len(payload)} bytes")

                if frame_num not in frame_buffers:
                    frame_buffers[frame_num] = {"total_chunks": total_chunks, "chunks": {}}
                frame_buffers[frame_num]["chunks"][chunk_idx] = payload
                buf = frame_buffers[frame_num]

                if len(buf["chunks"]) == buf["total_chunks"]:
                    try:
                        full_frame = b"".join(
                            buf["chunks"][i] for i in range(buf["total_chunks"])
                        )
                    except KeyError:
                        pass
                    else:
                        del frame_buffers[frame_num]
                        # Décoder JPEG ou H264
                        img = cv2.imdecode(
                            np.frombuffer(full_frame, dtype=np.uint8),
                            cv2.IMREAD_COLOR,
                        )
                        if img is None and HAS_AV and h264_decoder is not None:
                            try:
                                packet = av.Packet(full_frame)
                                for frame in h264_decoder.decode(packet):
                                    img = frame.to_ndarray(format="bgr24")
                                    break
                            except Exception:
                                pass
                        if img is not None:
                            frame_count += 1
                            last_frame_time = time.time()
                            # Initialiser VideoWriter à la première frame
                            if video_writer is None:
                                output_path = args.output or f"recording_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
                                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                                video_writer = cv2.VideoWriter(
                                    output_path,
                                    fourcc,
                                    args.fps,
                                    (img.shape[1], img.shape[0]),
                                )
                                print(f"[CameraReceiver] Enregistrement vers {output_path}")
                            video_writer.write(img)
                            if args.debug and frame_count <= 10:
                                print(f"[CameraReceiver] [debug] Frame {frame_num} décodée: {img.shape[1]}x{img.shape[0]}")
                            cv2.imshow("Robocar Camera", img)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                break
                        else:
                            if args.debug:
                                print(f"[CameraReceiver] [debug] Frame {frame_num}: décodage échoué (JPEG et H264)")
                    # Nettoyer les vieux buffers (éviter fuite mémoire)
                    old_frames = [
                        fn
                        for fn in frame_buffers
                        if fn < frame_num - 100
                    ]
                    for fn in old_frames:
                        del frame_buffers[fn]
            else:
                if args.debug and packet_count <= 20:
                    print(f"[CameraReceiver] [debug] JSON (status) ignoré: {data[:50]}...")

    except KeyboardInterrupt:
        print("\n[CameraReceiver] Arrêt demandé.")
    finally:
        sock.close()
        cv2.destroyAllWindows()
        if video_writer is not None:
            video_writer.release()
            print(f"[CameraReceiver] Vidéo sauvegardée: {output_path} ({frame_count} frames)")
        if args.debug:
            print(f"[CameraReceiver] [debug] Total: {packet_count} paquets reçus, {frame_count} frames décodées.")
        elif last_frame_time and video_writer is None:
            print(f"[CameraReceiver] {frame_count} frames reçues.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
