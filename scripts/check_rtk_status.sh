#!/bin/bash
#
# Script de diagnostic RTK - Vérifie si le récepteur RTK reçoit des corrections
#
# Usage:
#   ./scripts/check_rtk_status.sh [--port /dev/ttyUSB0] [--duration 30]
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
RTK_PORT="${1:-/dev/ttyUSB0}"
DURATION="${2:-30}"

# Chercher p1-host-tools dans le projet d'abord, puis dans $HOME
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

echo "=================================="
echo "DIAGNOSTIC RTK POLARIS"
echo "=================================="
echo ""

# Vérifier le port
if [ ! -e "$RTK_PORT" ]; then
    error "Port série $RTK_PORT introuvable"
    exit 1
fi
success "Port série: $RTK_PORT"

# Vérifier p1-host-tools
if [ ! -d "$P1_HOST_TOOLS_DIR" ]; then
    error "p1-host-tools non trouvé dans $P1_HOST_TOOLS_DIR"
    info "Recherche dans le projet..."
    if [ -d "$PROJECT_ROOT/p1-host-tools" ]; then
        P1_HOST_TOOLS_DIR="$PROJECT_ROOT/p1-host-tools"
        success "p1-host-tools trouvé dans le projet: $P1_HOST_TOOLS_DIR"
    else
        error "p1-host-tools introuvable"
        info "Installez-le avec: git clone https://github.com/PointOneNav/p1-host-tools.git"
        exit 1
    fi
fi

cd "$P1_HOST_TOOLS_DIR"

if [ ! -f "bin/config_tool.py" ]; then
    error "config_tool.py non trouvé"
    exit 1
fi

echo ""
info "Vérification du statut RTK pendant $DURATION secondes..."
info "Surveillez 'corrections=' et 'Type=' dans les messages"
echo ""

# Lancer config_tool.py et capturer les informations importantes
timeout "$DURATION" python3 bin/config_tool.py --port "$RTK_PORT" 2>&1 | \
    tee /tmp/rtk_check.log | \
    grep -E "(Type=|corrections=|Detected|Invalid|RTK|LLA=)" || true

echo ""
echo "=================================="
echo "RÉSUMÉ DU DIAGNOSTIC"
echo "=================================="

# Analyser les logs
if grep -q "corrections=0 B" /tmp/rtk_check.log; then
    warning "AUCUNE correction RTK reçue (corrections=0 B)"
    echo ""
    echo "Causes possibles:"
    echo "  1. Le récepteur n'est pas connecté à Internet (Wi-Fi ou cellulaire)"
    echo "  2. Le récepteur n'est pas configuré avec l'ID Polaris"
    echo "  3. Problème de connexion au réseau Polaris"
    echo ""
    echo "Solution:"
    echo "  ./scripts/setup_rtk_polaris.sh --port $RTK_PORT"
elif grep -q "corrections=" /tmp/rtk_check.log; then
    CORRECTIONS=$(grep "corrections=" /tmp/rtk_check.log | tail -1 | grep -oP 'corrections=\K[0-9]+')
    if [ "$CORRECTIONS" -gt 0 ]; then
        success "Corrections RTK reçues: $CORRECTIONS bytes"
    fi
fi

if grep -q "Type=Invalid" /tmp/rtk_check.log; then
    warning "Solution invalide détectée (Type=Invalid)"
    echo ""
    echo "Causes possibles:"
    echo "  1. Pas de vue du ciel (antenne à l'intérieur)"
    echo "  2. Pas de satellites visibles"
    echo "  3. Récepteur en cours d'initialisation (attendre 5-15 minutes)"
    echo "  4. Pas de corrections RTK reçues"
elif grep -q "Type=RTK" /tmp/rtk_check.log; then
    RTK_TYPE=$(grep "Type=" /tmp/rtk_check.log | tail -1 | grep -oP 'Type=\K[^ ]+')
    success "Solution RTK active: $RTK_TYPE"
fi

echo ""
info "Pour plus de détails, consultez: /tmp/rtk_check.log"
