#!/bin/bash
#
# Script pour configurer le récepteur RTK Polaris de manière persistante
# Configure le récepteur une fois avec le token Polaris, puis sauvegarde la configuration
#
# Usage:
#   ./scripts/configure_rtk_persistent.sh [--port /dev/ttyUSB1] [--device-id mon-device] [--duration 30]
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

# Valeurs par défaut
RTK_PORT="/dev/ttyUSB1"
DEVICE_ID="robocar-rtk"
DURATION=30

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

# Parser arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --port)
            RTK_PORT="$2"
            shift 2
            ;;
        --device-id)
            DEVICE_ID="$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --port PORT          Port série RTK (default: /dev/ttyUSB1)"
            echo "  --device-id ID       ID du device (default: robocar-rtk)"
            echo "  --duration SECONDS   Durée de configuration (default: 30)"
            echo ""
            exit 0
            ;;
        *)
            error "Option inconnue: $1"
            exit 1
            ;;
    esac
done

# Récupérer la clé Polaris depuis config.py
get_polaris_key() {
    local config_file="$PROJECT_ROOT/drive/core/config.py"
    if [ ! -f "$config_file" ]; then
        error "Fichier config.py introuvable"
        exit 1
    fi
    local key=$(grep -E "^POLARIS_API_KEY\s*=" "$config_file" | sed "s/.*['\"]\(.*\)['\"].*/\1/" | head -1)
    if [ -z "$key" ]; then
        error "POLARIS_API_KEY non trouvée"
        exit 1
    fi
    echo "$key"
}

echo "=================================="
echo "CONFIGURATION RTK POLARIS PERSISTANTE"
echo "=================================="
echo ""

POLARIS_KEY=$(get_polaris_key)
success "Clé Polaris: ${POLARIS_KEY:0:8}...${POLARIS_KEY: -8}"

# Vérifier p1-host-tools
if [ ! -d "$P1_HOST_TOOLS_DIR" ]; then
    error "p1-host-tools non trouvé"
    exit 1
fi

cd "$P1_HOST_TOOLS_DIR"

if [ ! -f "bin/runner.py" ]; then
    error "runner.py non trouvé"
    exit 1
fi

info "Configuration du récepteur RTK avec Polaris..."
info "  Port: $RTK_PORT"
info "  Device ID: $DEVICE_ID"
info "  Durée: $DURATION secondes"
echo ""

warning "IMPORTANT:"
info "  1. Le récepteur DOIT être connecté à Internet (Wi-Fi ou cellulaire)"
info "  2. Cette configuration établit la connexion Polaris et peut la sauvegarder"
info "  3. Après configuration, le récepteur devrait se reconnecter automatiquement"
echo ""

info "Lancement de runner.py pour configurer le récepteur..."
info "Surveillez 'corrections=' - cela doit augmenter si la connexion Polaris fonctionne"
info "Appuyez sur Ctrl+C après $DURATION secondes ou quand vous voyez des corrections RTK"
echo ""

# Lancer runner.py avec timeout
timeout "$DURATION" python3 bin/runner.py \
    --device-id "$DEVICE_ID" \
    --polaris "$POLARIS_KEY" \
    --port "$RTK_PORT" \
    2>&1 | tee /tmp/rtk_config.log || {
    # Timeout est normal, on continue
    true
}

echo ""
echo "=================================="
echo "RÉSUMÉ DE LA CONFIGURATION"
echo "=================================="

# Analyser les logs
if grep -q "corrections=" /tmp/rtk_config.log; then
    CORRECTIONS=$(grep "corrections=" /tmp/rtk_config.log | tail -1 | grep -oP 'corrections=\K[0-9]+' || echo "0")
    if [ "$CORRECTIONS" -gt 0 ]; then
        success "Corrections RTK reçues: $CORRECTIONS bytes"
        success "La connexion Polaris est établie!"
    else
        warning "Aucune correction RTK reçue (corrections=0)"
        warning "Vérifiez la connexion Internet du récepteur"
    fi
fi

if grep -q "Type=RTK\|Type=AutonomousGPS" /tmp/rtk_config.log; then
    RTK_TYPE=$(grep "Type=" /tmp/rtk_config.log | tail -1 | grep -oP 'Type=\K[^ ]+' || echo "Unknown")
    success "Solution GNSS active: $RTK_TYPE"
elif grep -q "Type=Invalid" /tmp/rtk_config.log; then
    warning "Solution toujours invalide - attendez quelques minutes pour la convergence"
fi

echo ""
info "Configuration terminée!"
info "Le récepteur devrait maintenant se connecter automatiquement à Polaris"
info "Testez avec: python scripts/test_rtk.py --port $RTK_PORT"
echo ""
