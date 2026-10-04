FROM ros:humble-ros-base-jammy AS dependencies
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs pkg-config curl ca-certificates \
    python3-pip python3-colcon-common-extensions python3-rosdep python3-pytest \
    python3-numpy python3-opencv python3-yaml python3-serial \
    libusb-1.0-0-dev libudev-dev libyaml-cpp-dev libboost-thread-dev \
    libgflags-dev libgoogle-glog-dev libdw-dev \
    ros-humble-asio-cmake-module \
    && rm -rf /var/lib/apt/lists/*

COPY requirements-hardware.txt /tmp/requirements-hardware.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements-hardware.txt

COPY src /tmp/hardware-dependencies/src
RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi && \
    rosdep update --rosdistro humble && \
    apt-get update && \
    rosdep install --from-paths /tmp/hardware-dependencies/src --ignore-src \
      --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/* /tmp/hardware-dependencies

FROM dependencies AS builder
WORKDIR /opt/duojin01
COPY src ./src
RUN source /opt/ros/humble/setup.bash && \
    colcon build --merge-install --parallel-workers 2 \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM dependencies AS runtime
COPY --from=builder /opt/duojin01/install /opt/duojin01/install
COPY docker/entrypoint.sh /usr/local/bin/duojin01-entrypoint
RUN chmod +x /usr/local/bin/duojin01-entrypoint && mkdir -p /ws/maps && \
    printf '%s\n' \
      'source /opt/ros/humble/setup.bash' \
      'source /opt/duojin01/install/setup.bash' \
      'if [[ -r /ws/install/hardware/setup.bash ]]; then source /ws/install/hardware/setup.bash; fi' \
      >> /root/.bashrc
ENV DUOJIN01_WORKSPACE_ROOT=/ws
ENV ROS_DOMAIN_ID=20
WORKDIR /ws
ENTRYPOINT ["/usr/local/bin/duojin01-entrypoint"]
CMD ["bash"]
