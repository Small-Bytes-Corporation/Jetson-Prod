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

import cv2
import numpy as np

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

    print(f"[CameraReceiver] Écoute sur le port UDP {args.port}...")
    print("[CameraReceiver] En attente de flux caméra... (Ctrl+C pour quitter)")

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[CameraReceiver] Erreur recvfrom: {e}")
                continue

            if len(data) < CHUNK_HEADER_SIZE:
                continue

            # Paquet chunké caméra (binaire)
            if data[0] != 0x7B:  # '{' = début JSON, on ignore les messages status
                try:
                    frame_num, total_chunks, chunk_idx = struct.unpack(
                        CHUNK_HEADER_FORMAT, data[:CHUNK_HEADER_SIZE]
                    )
                except struct.error:
                    continue
                payload = data[CHUNK_HEADER_SIZE:]

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
                        # Décoder JPEG (ou H264 si PyAV disponible)
                        img = cv2.imdecode(
                            np.frombuffer(full_frame, dtype=np.uint8),
                            cv2.IMREAD_COLOR,
                        )
                        if img is not None:
                            frame_count += 1
                            last_frame_time = time.time()
                            cv2.imshow("Robocar Camera", img)
                            if cv2.waitKey(1) & 0xFF == ord("q"):
                                break
                    # Nettoyer les vieux buffers (éviter fuite mémoire)
                    old_frames = [
                        fn
                        for fn in frame_buffers
                        if fn < frame_num - 100
                    ]
                    for fn in old_frames:
                        del frame_buffers[fn]

    except KeyboardInterrupt:
        print("\n[CameraReceiver] Arrêt demandé.")
    finally:
        sock.close()
        cv2.destroyAllWindows()
        if last_frame_time:
            print(f"[CameraReceiver] {frame_count} frames reçues.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
