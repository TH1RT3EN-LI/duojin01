FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# 强制 rosdep 用 USTC rosdistro index（比写 bashrc 更稳定）
ENV ROSDISTRO_INDEX_URL=https://mirrors.ustc.edu.cn/rosdistro/index-v4.yaml
ARG INSTALL_RUNTIME_DEPS=1
ARG ROSDEP_DEP_TYPES="buildtool build build_export buildtool_export exec"
# 0) Ubuntu apt 源切到 USTC（jammy=22.04）
RUN sed -i 's|http://archive.ubuntu.com/ubuntu|https://mirrors.ustc.edu.cn/ubuntu|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu|https://mirrors.ustc.edu.cn/ubuntu|g' /etc/apt/sources.list

# 1) 基础工具 + colcon + rosdep + 常用构建依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    git vim nano tmux \
    build-essential \
    curl ca-certificates \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rtree \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# 1.5) 增加 OSRF Gazebo 软件源（用于安装 Harmonic 对应的 ROS 包）
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# 用 USTC rosdistro 取代 rosdep 默认 sources（替 raw.githubusercontent.com）
RUN mkdir -p /etc/ros/rosdep/sources.list.d/ && \
    curl -fsSL -o /etc/ros/rosdep/sources.list.d/20-default.list \
      https://mirrors.ustc.edu.cn/rosdistro/rosdep/sources.list.d/20-default.list && \
    sed -i 's#raw.githubusercontent.com/ros/rosdistro/master#mirrors.ustc.edu.cn/rosdistro#g' \
      /etc/ros/rosdep/sources.list.d/20-default.list

# 2) package.xml 未定义的额外依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-asio-cmake-module \
    ros-humble-backward-ros \
    ros-humble-camera-info-manager \
    ros-humble-image-publisher \
    ros-humble-laser-filters \
    ros-humble-pcl-msgs \
    libboost-thread-dev \
    libyaml-cpp-dev \
    libdw-dev \
    libpcl-dev \
    python3-serial \
    && if [[ "${INSTALL_RUNTIME_DEPS}" == "1" ]]; then \
         apt-get install -y --no-install-recommends \
           ros-humble-teleop-twist-keyboard \
           ros-humble-ros-gzharmonic \
           gz-harmonic \
           libegl1-mesa \
           libgl1-mesa-dri \
           libglx-mesa0 \
           libgles2-mesa \
           libglu1-mesa \
           libosmesa6 \
           mesa-utils \
           libxkbcommon-x11-0 \
           libxcb-xinerama0 \
           libxcb-icccm4 \
           libxcb-image0 \
           libxcb-keysyms1 \
           libxcb-randr0 \
           libxcb-render-util0 \
           libxcb-xfixes0 \
           libxcb-shape0 \
           libxcb-cursor0 \
           libqt5gui5 \
           libqt5widgets5 \
           libqt5core5a \
           qtwayland5 \
           libqt5waylandclient5; \
       fi && \
    rm -rf /var/lib/apt/lists/*

# AprilTag detector backend used by duojin01_mission/april_tag_detector.py.
RUN if [[ "${INSTALL_RUNTIME_DEPS}" == "1" ]]; then \
      python3 -m pip install --no-cache-dir pupil-apriltags; \
    fi

WORKDIR /ws

# 3) rosdep init + update
RUN rosdep init || true && \
    for i in 1 2 3 4 5; do rosdep update && break || (echo "rosdep update failed, retry $i" && sleep 5); done

# 4) 仅在源码变化时重新计算工作区依赖
COPY ./src /ws/src

# 5) 按 ROSDEP_DEP_TYPES 安装工作区依赖；开发/仿真镜像建议包含 exec
RUN apt-get update && \
    dep_type_args=(); \
    for dep_type in ${ROSDEP_DEP_TYPES}; do \
      dep_type_args+=(--dependency-types "${dep_type}"); \
    done; \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble \
      "${dep_type_args[@]}" \
      --skip-keys "ros_gz_sim ros_gz_bridge foxglove_bridge" && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]
