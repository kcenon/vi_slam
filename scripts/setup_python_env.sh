#!/bin/bash
# Setup Python virtual environment

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PC_CLIENT_DIR="$PROJECT_ROOT/pc_client"
PYTHON_DIR="$PC_CLIENT_DIR/python"
VENV_DIR="$PYTHON_DIR/venv"

echo "Setting up Python virtual environment..."

# Check Python version
python3 --version || { echo "Python 3 is required"; exit 1; }

# Create virtual environment
python3 -m venv "$VENV_DIR"

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Upgrade pip
pip install --upgrade pip

# Install requirements
pip install -r "$PYTHON_DIR/requirements.txt"

echo ""
echo "Python environment setup completed!"
echo "To activate the environment, run:"
echo "  source $VENV_DIR/bin/activate"
