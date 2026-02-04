#!/bin/bash
#
# Script pour initialiser la connexion Polaris RTK une seule fois
# Configure le récepteur avec le token Polaris et établit la connexion initiale
# Le récepteur devrait ensuite se reconnecter automatiquement s'il a Internet intégré
#
# Usage:
#   ./scripts/init_rtk_polaris.sh [--port /dev/ttyUSB1] [--device-id mon-device]
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RTK_PORT="${1:-/dev/ttyUSB1}"
DEVICE_ID="${2:-robocar-rtk}"

# Chercher p1-host-tools
if [ -d "$PROJECT_ROOT/p1-host-tools" ]; then
    P1_HOST_TOOLS_DIR="$PROJECT_ROOT/p1-host-tools"
elif [ -d "$HOME/p1-host-tools" ]; then
    P1_HOST_TOOLS_DIR="$HOME/p1-host-tools"
else
    P1_HOST_TOOLS_DIR="$HOME/p1-host-tools"
fi

# Récupérer la clé Polaris depuis config.py
get_polaris_key() {
    local config_file="$PROJECT_ROOT/drive/core/config.py"
    grep -E "^POLARIS_API_KEY\s*=" "$config_file" | sed "s/.*['\"]\(.*\)['\"].*/\1/" | head -1
}

POLARIS_KEY=$(get_polaris_key)

if [ -z "$POLARIS_KEY" ]; then
    echo "ERREUR: Clé Polaris non trouvée dans config.py"
    exit 1
fi

echo "Initialisation de la connexion Polaris RTK..."
echo "Port: $RTK_PORT"
echo "Device ID: $DEVICE_ID"
echo "Polaris Key: ${POLARIS_KEY:0:8}...${POLARIS_KEY: -8}"
echo ""

cd "$P1_HOST_TOOLS_DIR"

# Vérifier/installer les dépendances
if ! python3 -c "import fusion_engine_client" 2>/dev/null; then
    echo "Installation des dépendances p1-host-tools..."
    pip install -q -r requirements.txt
fi

echo "Lancement de runner.py pour établir la connexion Polaris..."
echo "Appuyez sur Ctrl+C une fois que vous voyez 'corrections=X B' avec X > 0"
echo ""

# Lancer runner.py - l'utilisateur peut l'arrêter manuellement une fois la connexion établie
python3 bin/runner.py \
    --device-id "$DEVICE_ID" \
    --polaris "$POLARIS_KEY" \
    --port "$RTK_PORT"
