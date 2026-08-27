FROM ros:jazzy-ros-base-noble

ARG DEBIAN_FRONTEND=noninteractive
ARG CRAZYSWARM2_REF
ARG CRAZYFLIE_FIRMWARE_REF

SHELL ["/bin/bash", "-c"]

# ------------------------------------------------------------
# 1. Herramientas básicas + dependencias indicadas por upstream
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    usbutils \
    git \
    ca-certificates \
    python3-pip \
    python3-setuptools \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-dev \
    build-essential \
    swig \
    libboost-program-options-dev \
    libusb-1.0-0-dev \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# 
#
# ------------------------------------------------------------
# Python dependencies
#
# Docker is our isolated environment. Some Python packages
# already exist as Ubuntu/Debian packages, but Crazyswarm2
# and CFlib require specific versions.
#
# --break-system-packages:
#   allows pip installation in Ubuntu 24.04's managed Python.
#
# --ignore-installed:
#   does not try to uninstall Debian-managed packages.
#   The pinned versions are installed in /usr/local and take
#   precedence in Python's import path.
# ------------------------------------------------------------
COPY docker/requirements-python.txt /tmp/requirements-python.txt

RUN pip3 install \
        --break-system-packages \
        --ignore-installed \
        --no-cache-dir \
        -r /tmp/requirements-python.txt



# ------------------------------------------------------------
# CFlib ROS dependency
# ------------------------------------------------------------
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# 3. Workspace exclusivo para Crazyswarm2 upstream
# ------------------------------------------------------------
RUN mkdir -p /opt/crazyswarm2_ws/src


# ------------------------------------------------------------
# 4. Clonar Crazyswarm2 y hacer checkout DEL COMMIT EXACTO
# ------------------------------------------------------------
RUN test -n "${CRAZYSWARM2_REF}" \
    && git clone \
        https://github.com/IMRCLab/crazyswarm2.git \
        /opt/crazyswarm2_ws/src/crazyswarm2 \
    && cd /opt/crazyswarm2_ws/src/crazyswarm2 \
    && git checkout "${CRAZYSWARM2_REF}" \
    && git submodule update --init --recursive \
    && echo "Pinned Crazyswarm2 commit:" \
    && git rev-parse HEAD


# ------------------------------------------------------------
# 5. Instalar dependencias ROS declaradas por los paquetes
#
# En lugar de intentar adivinar manualmente cada dependencia,
# rosdep lee los package.xml del commit que acabamos de fijar.
# ------------------------------------------------------------
RUN rosdep init 2>/dev/null || true \
    && rosdep update \
    && apt-get update \
    && cd /opt/crazyswarm2_ws \
    && source /opt/ros/jazzy/setup.bash \
    && rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro jazzy \
        -r \
        -y \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# 6. Motion capture runtime
#
# Crazyswarm2 utiliza motion_capture_tracking desde launch.py,
# pero el nodo completo no está declarado como dependencia
# rosdep de los paquetes del workspace.
#
# La documentación oficial de Crazyswarm2 pide instalarlo
# explícitamente.
# ------------------------------------------------------------
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-motion-capture-tracking \
    && rm -rf /var/lib/apt/lists/*


# ------------------------------------------------------------
# 7. Compilar tal como indica la documentación oficial
# ------------------------------------------------------------
RUN cd /opt/crazyswarm2_ws \
    && source /opt/ros/jazzy/setup.bash \
    && colcon build \
        --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release



# ------------------------------------------------------------
# Crazyflie firmware Python bindings
#
# Required by the Crazyswarm2 software-in-the-loop simulator.
# ------------------------------------------------------------
RUN test -n "${CRAZYFLIE_FIRMWARE_REF}" \
    && git clone \
        https://github.com/bitcraze/crazyflie-firmware.git \
        /opt/crazyflie-firmware \
    && cd /opt/crazyflie-firmware \
    && git checkout "${CRAZYFLIE_FIRMWARE_REF}" \
    && git submodule update --init --recursive \
    && echo "Pinned Crazyflie firmware commit:" \
    && git rev-parse HEAD \
    && make cf2_defconfig \
    && make bindings_python

ENV PYTHONPATH=/opt/crazyflie-firmware/build

RUN python3 -c \
    "import cffirmware, _cffirmware; \
     print('cffirmware OK:', cffirmware.__file__); \
     print('_cffirmware OK:', _cffirmware.__file__)"



# ------------------------------------------------------------
# Visualization
# ------------------------------------------------------------
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*




    
WORKDIR /opt/crazyswarm2_ws

CMD ["/bin/bash"]
