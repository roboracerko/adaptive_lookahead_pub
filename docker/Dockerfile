FROM ros:humble-ros-base-jammy

# 시스템 의존성
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ackermann-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-map-server \
    ros-humble-nav2-map-server \
    ros-humble-lifecycle-msgs \
    ros-humble-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*

# Python 의존성
RUN pip3 install --no-cache-dir \
    numpy scipy matplotlib pandas pyyaml pillow \
    f1tenth-gym

# 워크스페이스 설정
WORKDIR /workspace
COPY . /workspace/

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"

# 환경 설정
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
