FROM px4io/px4-dev-ros-noetic

# Nvidia GPU vars
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute,display

# Change according your SDK version
ENV VULKAN_SDK_VERSION v1.2.194

# Host user credetentials
ENV uid 1002
ENV gid 100
ENV USER user

# Comatibility libs
RUN apt-get update && apt-get install -y \
    apt-utils \
    libxext6 \
    libx11-6 \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libfuse-dev \
    fuse \
    libpulse-mainloop-glib0 \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \ 
    gstreamer1.0-gl \
    cmake \
    build-essential \
    libx11-xcb-dev \
    libxkbcommon-dev \
    libwayland-dev \
    libxrandr-dev \
    git \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt  /lists/*

# Vulkan support
RUN git clone --depth 1 --branch ${VULKAN_SDK_VERSION} https://github.com/KhronosGroup/Vulkan-Headers.git /opt/vulkan/headers && \
    cd /opt/vulkan/headers && \
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=install .. && \
    make install
RUN ln -s /usr/bin/python3 /usr/bin/python && \   
    git clone --depth 1 --branch ${VULKAN_SDK_VERSION} https://github.com/KhronosGroup/Vulkan-Tools.git /opt/vulkan/tools && \
    cd /opt/vulkan/tools && \
    mkdir build && cd build && ../scripts/update_deps.py &&\
    cmake -DCMAKE_BUILD_TYPE=Release -C helper.cmake .. && cmake --build . && \
    make install
# Hardcoded ICD profiles, please refer to your driver version
COPY NVIDIA-Linux-x86_64-470.62.07/nvidia_icd.json /etc/vulkan/icd.d/nvidia_icd.json
COPY NVIDIA-Linux-x86_64-470.62.07/nvidia_layers.json /etc/vulkan/implicit_layer.d/nvidia_layers.json
COPY NVIDIA-Linux-x86_64-470.62.07/10_nvidia.json /usr/share/glvnd/egl_vendor.d/10_nvidia.json

# User vars
RUN useradd -ms /bin/bash --uid ${uid} --gid ${gid} ${USER} && \
    usermod -aG sudo ${USER}
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER ${USER}
ENV HOME /home/${USER}

# Drone Games repo install
WORKDIR /home/${USER}
RUN touch .bashrc && git clone --single-branch https://github.com/acsl-mipt/drone-games.git
WORKDIR /home/${USER}/drone-games
RUN ./install.sh && rosdep update

# PX4 Gazebo var
RUN printf "export QT_GRAPHICSSYSTEM=native" >> /home/${USER}/.bashrc
