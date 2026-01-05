#!/bin/bash

set -e

VENV_NAME="ros2_torch"
VENV_PATH="${HOME}/${VENV_NAME}"

echo "========================================="
echo "ROS2 + PyTorch Environment Setup Script"
echo "========================================="
echo ""

# Check if virtual environment already exists
if [ -d "$VENV_PATH" ]; then
    echo "Warning: Virtual environment '${VENV_NAME}' already exists at ${VENV_PATH}"
    read -p "Do you want to remove it and create a new one? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Removing existing virtual environment..."
        rm -rf "$VENV_PATH"
    else
        echo "Aborting setup."
        exit 0
    fi
fi

# Check Python version
echo "Checking Python version..."
PYTHON_CMD=$(command -v python3.12)
if [ -z "$PYTHON_CMD" ]; then
    echo "Error: python3.12 not found. Please install Python 3.12."
    echo "On Ubuntu/Debian: sudo apt install python3.12 python3.12-venv"
    exit 1
fi

PYTHON_VERSION=$($PYTHON_CMD --version 2>&1 | awk '{print $2}')
echo "Using Python version: $PYTHON_VERSION"

# Create virtual environment
echo ""
echo "Creating virtual environment '${VENV_NAME}'..."
$PYTHON_CMD -m venv "$VENV_PATH"

# Activate virtual environment
echo "Activating virtual environment..."
source "${VENV_PATH}/bin/activate"

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip setuptools wheel

# Detect GPU availability
echo ""
echo "========================================="
echo "Detecting GPU availability..."
echo "========================================="

HAS_GPU=false
CUDA_VERSION=""

if command -v nvidia-smi &> /dev/null; then
    echo "NVIDIA GPU detected!"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

    # Try to detect CUDA version
    if command -v nvcc &> /dev/null; then
        CUDA_VERSION=$(nvcc --version | grep "release" | sed -n 's/.*release \([0-9]\+\.[0-9]\+\).*/\1/p')
        echo "CUDA version detected: $CUDA_VERSION"
    else
        # Try to get CUDA version from nvidia-smi
        CUDA_VERSION=$(nvidia-smi | grep "CUDA Version" | sed -n 's/.*CUDA Version: \([0-9]\+\.[0-9]\+\).*/\1/p')
        if [ -n "$CUDA_VERSION" ]; then
            echo "CUDA version from nvidia-smi: $CUDA_VERSION"
        else
            echo "Warning: Could not detect CUDA version, will attempt to install CUDA-enabled PyTorch"
        fi
    fi

    HAS_GPU=true
else
    echo "No NVIDIA GPU detected. Will install CPU-only version of PyTorch."
fi

# Install PyTorch
echo ""
echo "========================================="
echo "Installing PyTorch..."
echo "========================================="

if [ "$HAS_GPU" = true ]; then
    echo "Installing PyTorch with CUDA support..."

    # Determine CUDA version for PyTorch
    if [ -n "$CUDA_VERSION" ]; then
        CUDA_MAJOR=$(echo $CUDA_VERSION | cut -d. -f1)
        CUDA_MINOR=$(echo $CUDA_VERSION | cut -d. -f2)

        # Map CUDA version to PyTorch CUDA version
        if [ "$CUDA_MAJOR" -ge 12 ]; then
            TORCH_CUDA="cu121"
            echo "Using PyTorch with CUDA 12.1 support"
        elif [ "$CUDA_MAJOR" -eq 11 ] && [ "$CUDA_MINOR" -ge 8 ]; then
            TORCH_CUDA="cu118"
            echo "Using PyTorch with CUDA 11.8 support"
        else
            TORCH_CUDA="cu118"
            echo "Using PyTorch with CUDA 11.8 support (fallback)"
        fi
    else
        TORCH_CUDA="cu121"
        echo "Using PyTorch with CUDA 12.1 support (default)"
    fi

    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/${TORCH_CUDA}

    # Verify GPU installation
    echo ""
    echo "Verifying PyTorch GPU installation..."
    python -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}'); print(f'CUDA version: {torch.version.cuda}'); print(f'Number of GPUs: {torch.cuda.device_count()}'); print(f'GPU name: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else \"N/A\"}')"

    if python -c "import torch; exit(0 if torch.cuda.is_available() else 1)"; then
        echo ""
        echo "SUCCESS: PyTorch with GPU support installed successfully!"
    else
        echo ""
        echo "WARNING: PyTorch installed but GPU support not available."
        echo "This could be due to driver issues or CUDA version mismatch."
        echo "The CPU version will still work, but will be slower."
    fi
else
    echo "Installing CPU-only version of PyTorch..."
    pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

    echo ""
    echo "Verifying PyTorch CPU installation..."
    python -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"
fi

# Install other Python dependencies
echo ""
echo "========================================="
echo "Installing Python dependencies..."
echo "========================================="

# Note: numpy 1.26.4 is compatible with Python 3.10+ and PyTorch 2.x
pip install numpy==1.26.4 scipy opencv-python pyyaml

# Install ROS2 Python dependencies (if not already available from system)
echo ""
echo "Installing additional dependencies..."
pip install setuptools

echo ""
echo "========================================="
echo "Setup Complete!"
echo "========================================="
echo ""
echo "Virtual environment '${VENV_NAME}' has been created at: ${VENV_PATH}"
echo ""
echo "To activate the environment, run:"
echo "    source ${VENV_PATH}/bin/activate"
echo ""
echo "To deactivate the environment, run:"
echo "    deactivate"
echo ""

if [ "$HAS_GPU" = true ]; then
    echo "PyTorch with GPU support has been installed."
    echo "Your GPU will be used for neural network inference."
else
    echo "PyTorch CPU-only version has been installed."
    echo "Neural network inference will run on CPU."
fi

echo ""
echo "Next steps:"
echo "1. Build the ROS2 package (do NOT activate venv for building):"
echo "   cd ~/sru_ws"
echo "   colcon build --packages-select rl_nav_controller --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "   Note: Do NOT use --symlink-install flag for the venv to work properly with PyTorch"
echo ""
echo "2. To run the navigation controller:"
echo "   source ${VENV_PATH}/bin/activate"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source ~/sru_ws/install/setup.bash"
echo "   ros2 launch rl_nav_controller rl_nav_controller.launch.py"
echo ""
