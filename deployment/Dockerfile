FROM ros:jazzy-perception

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC

WORKDIR /workspace 
#/Perception

# System tools + ROS dev basics (keep system python for ROS, but we’ll use a venv for ML deps)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        direnv \
        build-essential \
        gcc g++ make cmake ninja-build \
        git \
        python3 python3-dev python3-pip python3-venv \
        python3-rosdep \
        ros-dev-tools \
        lcov \
        libgl1 \
        libglib2.0-0 \
        libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Optional "python" alias
RUN ln -sf /usr/bin/python3 /usr/bin/python || true

# Create venv and make it the default python/pip/colcon in this container
RUN python3 -m venv /venv
ENV PATH="/venv/bin:$PATH"

# Install build tooling inside the venv so colcon uses venv python
RUN pip install --upgrade pip setuptools wheel && \
    pip install \
      colcon-common-extensions \
      vcstool \
      rosdep

# Install your Python dependencies into the venv (so ROS nodes can import them)
RUN pip install \
        pytest pytest-md pytest-emoji pytest-cov coverage \
        ruff \
        pybind11 pybind11-stubgen scikit-build-core \
        numpy scipy pandas matplotlib \
        scikit-learn \
        opencv-python \
        open3d

# optional
RUN pip install ultralytics ultralytics-v11
RUN pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision 

# Shell setup: ROS + venv enabled automatically
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /venv/bin/activate' >> /root/.bashrc && \
    echo 'export AMENT_PYTHON_EXECUTABLE=/venv/bin/python3' >> /root/.bashrc && \
    echo 'export PYTHON_EXECUTABLE=/venv/bin/python3' >> /root/.bashrc && \
    echo 'eval "$(direnv hook bash)"' >> /root/.bashrc && \
    echo 'cd /workspace' >> /root/.bashrc

CMD ["/bin/bash"]
