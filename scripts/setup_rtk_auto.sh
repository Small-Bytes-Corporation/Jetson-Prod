#!/bin/bash
#
# Script pour configurer le récepteur RTK pour se connecter automatiquement à Polaris
# Si le récepteur a une connexion Internet intégrée, cette configuration devrait permettre
# une connexion automatique sans avoir besoin de runner.py en continu
#
# Usage:
#   ./scripts/setup_rtk_auto.sh [--port /dev/ttyUSB1]
#

set -e

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

RTK_PORT="${1:-/dev/ttyUSB1}"

# Chercher p1-host-tools
if [ -d "$PROJECT_ROOT/p1-host-tools" ]; then
    P1_HOST_TOOLS_DIR="$PROJECT_ROOT/p1-host-tools"
elif [ -d "$HOME/p1-host-tools" ]; then
    P1_HOST_TOOLS_DIR="$HOME/p1-host-tools"
else
    P1_HOST_TOOLS_DIR="$HOME/p1-host-tools"
fi

info() { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[OK]${NC} $1"; }
warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Récupérer la clé Polaris
get_polaris_key() {
    local config_file="$PROJECT_ROOT/drive/core/config.py"
    grep -E "^POLARIS_API_KEY\s*=" "$config_file" | sed "s/.*['\"]\(.*\)['\"].*/\1/" | head -1
}

echo "=================================="
echo "CONFIGURATION RTK POLARIS AUTOMATIQUE"
echo "=================================="
echo ""

POLARIS_KEY=$(get_polaris_key)
if [ -z "$POLARIS_KEY" ]; then
    error "Clé Polaris non trouvée dans config.py"
    exit 1
fi

success "Clé Polaris: ${POLARIS_KEY:0:8}...${POLARIS_KEY: -8}"
info "Port: $RTK_PORT"
echo ""

warning "NOTE IMPORTANTE:"
info "Si votre récepteur RTK a une connexion Internet intégrée (Wi-Fi ou cellulaire),"
info "il peut se connecter directement à Polaris après configuration."
info ""
info "Si le récepteur n'a PAS de connexion Internet intégrée, vous DEVEZ utiliser"
info "runner.py en continu pour faire le pont entre Polaris et le récepteur."
echo ""

# Vérifier p1-host-tools
if [ ! -d "$P1_HOST_TOOLS_DIR" ]; then
    error "p1-host-tools non trouvé"
    exit 1
fi

cd "$P1_HOST_TOOLS_DIR"

# Option 1: Utiliser config_tool.py pour sauvegarder la configuration
if [ -f "bin/config_tool.py" ]; then
    info "Option 1: Sauvegarder la configuration avec config_tool.py"
    info "Tentative de sauvegarde de la configuration actuelle..."
    
    if python3 bin/config_tool.py --device "$RTK_PORT" save 2>&1 | grep -q "saved\|success"; then
        success "Configuration sauvegardée!"
    else
        warning "La sauvegarde de configuration a peut-être échoué ou n'est pas nécessaire"
    fi
    echo ""
fi

# Option 2: Lancer runner.py brièvement pour établir la connexion
info "Option 2: Établir la connexion Polaris avec runner.py"
info "Lancement de runner.py pendant 10 secondes pour établir la connexion..."
info "Si le récepteur a une connexion Internet intégrée, il devrait se souvenir de cette configuration"
echo ""

timeout 10 python3 bin/runner.py \
    --device-id "robocar-rtk" \
    --polaris "$POLARIS_KEY" \
    --port "$RTK_PORT" \
    2>&1 | tee /tmp/rtk_auto_config.log || true

echo ""
echo "=================================="
echo "RÉSUMÉ"
echo "=================================="

if grep -q "corrections=" /tmp/rtk_auto_config.log; then
    CORRECTIONS=$(grep "corrections=" /tmp/rtk_auto_config.log | tail -1 | grep -oP 'corrections=\K[0-9]+' || echo "0")
    if [ "$CORRECTIONS" -gt 0 ]; then
        success "Connexion Polaris établie! Corrections reçues: $CORRECTIONS bytes"
    fi
fi

echo ""
info "Testez maintenant votre application Python:"
info "  python3 main.py --rtk-port $RTK_PORT --enable-socket"
echo ""
info "Si vous voyez toujours des NaN, le récepteur n'a probablement pas de connexion Internet intégrée."
info "Dans ce cas, vous devez utiliser runner.py en continu:"
info "  cd $P1_HOST_TOOLS_DIR"
info "  python3 bin/runner.py --device-id robocar-rtk --polaris $POLARIS_KEY --port $RTK_PORT &"
echo ""
