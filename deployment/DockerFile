# ROS 2 Jazzy base image (Ubuntu 24.04 / Noble)
# "jazzy" tag is an alias for jazzy-ros-base, see official docs.
# https://hub.docker.com/_/ros
FROM ros:jazzy-perception

# Avoid interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC

WORKDIR /workspace

# --- Cute banner ---
RUN echo "🌟 Welcome to the ROS 2 Jazzy + ML Dev Environment! 🌟" && \
    echo "🚀 Setting up ROS 2, tools and ML libraries..."

# --- System packages & ROS dev tools ---
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        # Shell / QoL
        bash-completion \
        direnv \
        # Build tools
        build-essential \
        gcc g++ make cmake ninja-build \
        git \
        # Python
        python3 python3-dev python3-pip python3-venv \
        # ROS dev tools (colcon, rosdep, etc.)
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-dev-tools \
        # Coverage + deps
        lcov \
        # For OpenCV / Open3D visualization
        libgl1 \
        libglib2.0-0 \
        # Crypto / TLS
        libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Optional: "python" alias
RUN ln -s /usr/bin/python3 /usr/bin/python || true

# --- Python virtual environment ---
RUN python3 -m venv /venv
ENV PATH="/venv/bin:$PATH"

# --- Python packages: testing, tooling, ML, CV, YOLO11 ---
RUN pip install --upgrade pip && \
    pip install \
        # Testing / quality
        pytest pytest-md pytest-emoji pytest-cov coverage \
        ruff \
        # Build / bindings
        pybind11 pybind11-stubgen setuptools scikit-build-core \
        # Numeric / ML stack
        numpy scipy pandas matplotlib \
        scikit-learn \
        # Computer vision
        opencv-python \
        # YOLO11 (via Ultralytics)
        ultralytics \
        ultralytics-v11

# If you want PyTorch CPU wheels explicitly, uncomment and adjust versions if needed:
# RUN pip install --index-url https://download.pytorch.org/whl/cpu \
#        torch torchvision torchaudio

# --- ROS 2 + dev environment in shell ---
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /venv/bin/activate' >> /root/.bashrc && \
    echo 'eval "$(direnv hook bash)"' >> /root/.bashrc && \
    echo '' >> /root/.bashrc && \
    echo 'cd /workspace' >> /root/.bashrc && \
    echo 'if [ ! -f .envrc ] && [ -f deployment/.envrc.default ]; then' >> /root/.bashrc && \
    echo '    cp deployment/.envrc.default .envrc' >> /root/.bashrc && \
    echo '    direnv allow' >> /root/.bashrc && \
    echo 'fi' >> /root/.bashrc

# Final build message
RUN echo "🎉 ROS 2 Jazzy + ML setup complete! Happy coding! 🎉"

# Default shell
CMD ["/bin/bash"]