#!/bin/bash
USER="vroum" # Nom d'utilisateur sur la Jetson
PROJECT_DIR="Jetson-Prod" # Nom du dossier contenant l'env virtuel
ENV_DIR="car/bin/activate" # Chemin vers l'activate à l'intérieur du dossier

if [ -z "$1" ]; then
  echo "[USAGE] $0 <IP>"
  exit 1
fi

ip="$1"
echo "[INFO] Connexion à la Jetson ($ip)..."
ssh -t $USER@$ip << EOF
cd ~/$PROJECT_DIR || exit 1
echo "[INFO] Activation de l'env python."
source $ENV_DIR
echo "[INFO] Git pull du repertoire SBC"
git pull
EOF
