#!/bin/bash
# Created by Nelson Durrant, Oct 2024
#
# Manages Docker containers locally
#
# DETAILS:
# - Starts/stops `marsrover` container using docker-compose
# - Forces rebuild on every 'up' command
#
# DEPENDENCIES:
# - docker/docker-compose.yml
# - scripts/entrypoint.sh
# CALLED BY: USER

set -e

# -----------------------
# Configuration
# -----------------------
LOCAL_DOCKER_COMPOSE_FILE="$(dirname "$(realpath "$0")")/docker_scan_slam/docker-compose.yaml"

# -----------------------
# Helper functions
# -----------------------
function printInfo {
    echo -e "\033[0m\033[36m[INFO] $1\033[0m" >&2
}

function printWarning {
    echo -e "\033[0m\033[33m[WARNING] $1\033[0m" >&2
}

function printError {
    echo -e "\033[0m\033[31m[ERROR] $1\033[0m" >&2
}

# -----------------------
# Docker operations
# -----------------------
function docker_up_local() {
    printInfo "Building and starting marsrover container..."
    docker compose -f "$LOCAL_DOCKER_COMPOSE_FILE" up -d --force-recreate --build
}

function docker_down_local() {
    printWarning "Stopping marsrover container..."
    docker compose -f "$LOCAL_DOCKER_COMPOSE_FILE" down
}

# -----------------------
# Argument parsing
# -----------------------
if [[ $# -eq 0 ]]; then
    printError "No action specified!"
    echo "Usage: $0 <up|down>"
    echo ""
    echo "Actions:"
    echo "  up    : Build and start Docker container (forces rebuild)"
    echo "  down  : Stop Docker container"
    exit 1
fi

ACTION="$1"

case "$ACTION" in
    "up")
        docker_up_local
        printInfo "Container is up!"
        ;;
    "down")
        docker_down_local
        printInfo "Container is down!"
        ;;
    "--help"|"-h")
        echo "Usage: $0 <up|down>"
        echo ""
        echo "Actions:"
        echo "  up    : Build and start Docker container (forces rebuild)"
        echo "  down  : Stop Docker container"
        exit 0
        ;;
    *)
        printError "Unknown action: $ACTION"
        echo "Usage: $0 <up|down>"
        echo "Use --help for more information"
        exit 1
        ;;
esac