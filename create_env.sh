#!/bin/bash

set -e
echo "Création de l'environnement virtuel..."
python3 -m venv car
source car/bin/activate
pip install --upgrade pip
echo "Installation des dépendances..."
pip install \
  crccheck==1.3.0 \
  pygame==2.6.1 \
  pyserial==3.5 \
  pythoncrc==1.21
pip install git+https://github.com/LiamBindle/pyvesc.git
echo "Sauvegarde dans requirements.txt..."
pip freeze > requirements.txt
echo "Environnement prêt !"
