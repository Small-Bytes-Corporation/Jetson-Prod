#!/bin/bash

USER="user"
SUBSTRING="jetson"
SUBNET="192.168.1.0/24"

echo "[INFO] Scan du réseau avec nmap..."
ACTIVE_IPS=$(nmap -sn $SUBNET | grep "Nmap scan report for" | awk '{print $5}')

echo "[INFO] IPs actives détectées :"
echo "$ACTIVE_IPS"
echo
for ip in $ACTIVE_IPS; do
  echo -n "Test de $ip ... "
  hostname=$(ssh -o ConnectTimeout=1 \
               -o StrictHostKeyChecking=no \
               -o BatchMode=yes \
               $USER@$ip "hostname" 2>/dev/null)
  if [[ "$hostname" == *$SUBSTRING* ]]; then
    echo "[OK] Jetson trouvée à l'adresse : $ip (hostname = $hostname)"
    echo "$ip" > last_jetson_ip.txt
    exit 0
  else
    echo "pas Jetson"
  fi
done
echo "[ERREUR] Aucune Jetson trouvée."
exit 1
