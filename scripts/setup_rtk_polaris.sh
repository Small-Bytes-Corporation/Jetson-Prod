#!/bin/bash
#
# Script de configuration RTK Polaris pour Point One Navigation
# Configure automatiquement le récepteur RTK avec l'ID Polaris depuis config.py
#
# Usage:
#   ./scripts/setup_rtk_polaris.sh [--port /dev/ttyUSB0] [--device-id MON_DEVICE] [--check-only]
#

set -e  # Arrêter en cas d'erreur

# Couleurs pour les messages
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Répertoire du script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Valeurs par défaut
RTK_PORT="/dev/ttyUSB0"
DEVICE_ID="robocar-rtk"
CHECK_ONLY=false

# Chercher p1-host-tools dans le projet d'abord, puis dans $HOME
if [ -d "$PROJECT_ROOT/p1-host-tools" ]; then
    P1_HOST_TOOLS_DIR="$PROJECT_ROOT/p1-host-tools"
elif [ -d "$HOME/p1-host-tools" ]; then
    P1_HOST_TOOLS_DIR="$HOME/p1-host-tools"
else
    P1_HOST_TOOLS_DIR="$HOME/p1-host-tools"
fi

# Parser les arguments
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
        --check-only)
            CHECK_ONLY=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --port PORT          Port série RTK (default: /dev/ttyUSB0)"
            echo "  --device-id ID       ID du device (default: robocar-rtk)"
            echo "  --check-only         Vérifier seulement la connexion, ne pas configurer"
            echo "  --help, -h           Afficher cette aide"
            echo ""
            exit 0
            ;;
        *)
            echo -e "${RED}Erreur: Option inconnue: $1${NC}"
            echo "Utilisez --help pour voir les options disponibles"
            exit 1
            ;;
    esac
done

# Fonction pour afficher les messages
info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Fonction pour extraire la clé Polaris depuis config.py
get_polaris_key() {
    local config_file="$PROJECT_ROOT/drive/core/config.py"
    if [ ! -f "$config_file" ]; then
        error "Fichier config.py introuvable: $config_file"
        exit 1
    fi
    
    # Extraire POLARIS_API_KEY depuis config.py
    local key=$(grep -E "^POLARIS_API_KEY\s*=" "$config_file" | sed "s/.*['\"]\(.*\)['\"].*/\1/" | head -1)
    
    if [ -z "$key" ]; then
        error "POLARIS_API_KEY non trouvée dans config.py"
        exit 1
    fi
    
    echo "$key"
}

# Vérifier que Python est disponible
if ! command -v python3 &> /dev/null; then
    error "Python3 n'est pas installé"
    exit 1
fi

info "Python3 trouvé: $(python3 --version)"

# Vérifier que le port série existe
if [ ! -e "$RTK_PORT" ]; then
    warning "Le port série $RTK_PORT n'existe pas"
    info "Ports série disponibles:"
    ls -1 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  Aucun port trouvé"
    exit 1
fi

success "Port série trouvé: $RTK_PORT"

# Récupérer la clé Polaris
info "Récupération de la clé Polaris depuis config.py..."
POLARIS_KEY=$(get_polaris_key)
success "Clé Polaris récupérée: ${POLARIS_KEY:0:8}...${POLARIS_KEY: -8}"

if [ "$CHECK_ONLY" = true ]; then
    info "Mode vérification uniquement"
    info "Vérification de la connexion au récepteur RTK..."
    
    if [ -d "$P1_HOST_TOOLS_DIR" ]; then
        cd "$P1_HOST_TOOLS_DIR"
        if [ -f "bin/config_tool.py" ]; then
            info "Test de connexion avec config_tool.py (5 secondes)..."
            echo ""
            timeout 5 python3 bin/config_tool.py --port "$RTK_PORT" 2>&1 | grep -E "(Type=|corrections=|Detected|Invalid|RTK)" || true
            echo ""
            info "Vérifiez:"
            info "  - 'corrections=X B' doit être > 0 si le récepteur reçoit des corrections RTK"
            info "  - 'Type=' doit être 'RTK_FIX' ou 'RTK_FLOAT' (pas 'Invalid')"
        else
            warning "config_tool.py non trouvé dans $P1_HOST_TOOLS_DIR"
        fi
    else
        warning "p1-host-tools non trouvé dans $P1_HOST_TOOLS_DIR"
        info "Installez p1-host-tools pour vérifier la connexion"
    fi
    exit 0
fi

# Vérifier/installer p1-host-tools
if [ ! -d "$P1_HOST_TOOLS_DIR" ]; then
    info "p1-host-tools non trouvé, clonage du dépôt..."
    # Essayer de cloner dans le projet d'abord
    if [ -w "$PROJECT_ROOT" ]; then
        cd "$PROJECT_ROOT"
        P1_HOST_TOOLS_DIR="$PROJECT_ROOT/p1-host-tools"
    else
        cd "$HOME"
        P1_HOST_TOOLS_DIR="$HOME/p1-host-tools"
    fi
    
    if ! git clone https://github.com/PointOneNav/p1-host-tools.git 2>/dev/null; then
        error "Échec du clonage de p1-host-tools"
        exit 1
    fi
    success "p1-host-tools cloné dans $P1_HOST_TOOLS_DIR"
else
    info "p1-host-tools trouvé dans $P1_HOST_TOOLS_DIR"
fi

cd "$P1_HOST_TOOLS_DIR"

# Vérifier/installer les dépendances
info "Vérification des dépendances Python..."
if [ -f "requirements.txt" ]; then
    if ! python3 -c "import fusion_engine_client" 2>/dev/null; then
        info "Installation des dépendances p1-host-tools..."
        pip install -q -r requirements.txt
        success "Dépendances installées"
    else
        success "Dépendances déjà installées"
    fi
else
    warning "requirements.txt non trouvé"
fi

# Vérifier que runner.py existe
if [ ! -f "bin/runner.py" ]; then
    error "runner.py non trouvé dans $P1_HOST_TOOLS_DIR/bin/"
    exit 1
fi

# Configuration du récepteur RTK avec Polaris
info "Configuration du récepteur RTK avec Polaris..."
info "  Port: $RTK_PORT"
info "  Device ID: $DEVICE_ID"
info "  Polaris Key: ${POLARIS_KEY:0:8}...${POLARIS_KEY: -8}"

echo ""
warning "IMPORTANT: Pour que le récepteur RTK fonctionne, il DOIT être:"
info "  1. Connecté à Internet (Wi-Fi ou cellulaire)"
info "  2. Avec une vue du ciel (antenne à l'extérieur)"
info "  3. Attendre 5-15 minutes pour la convergence (cold start)"
echo ""

# Vérifier si config_tool.py existe pour configuration interactive
if [ -f "bin/config_tool.py" ]; then
    info "Option 1: Configuration interactive avec config_tool.py"
    info "  Utilisez config_tool.py pour configurer le récepteur manuellement"
    echo ""
fi

info "Option 2: Utilisation de runner.py avec Polaris"
info "  runner.py va se connecter au récepteur et utiliser l'ID Polaris"
info "  pour recevoir les corrections RTK du réseau Polaris"
echo ""

read -p "Voulez-vous lancer runner.py maintenant? (o/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Oo]$ ]]; then
    info "Lancement de runner.py..."
    info "Surveillez les messages pour voir si 'corrections=X B' augmente"
    info "et si 'Type=' passe de 'Invalid' à 'RTK_FIX' ou 'RTK_FLOAT'"
    info "Appuyez sur Ctrl+C pour arrêter"
    echo ""
    
    # Lancer runner.py avec les paramètres Polaris
    python3 bin/runner.py \
        --device-id "$DEVICE_ID" \
        --polaris "$POLARIS_KEY" \
        --port "$RTK_PORT" \
        || {
            warning "runner.py s'est arrêté"
            echo ""
            info "Si vous voyez toujours 'Type=Invalid', vérifiez:"
            info "  - Le récepteur est connecté à Internet"
            info "  - L'antenne a une vue du ciel"
            info "  - Attendez quelques minutes pour la convergence"
            exit 1
        }
    
    success "Configuration terminée!"
else
    info "Configuration annulée"
    echo ""
    info "Pour configurer manuellement:"
    info "  1. Utilisez config_tool.py: python bin/config_tool.py"
    info "  2. Ou utilisez runner.py: python bin/runner.py --device-id $DEVICE_ID --polaris $POLARIS_KEY --port $RTK_PORT"
    exit 0
fi
