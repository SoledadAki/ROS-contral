# C21 runtime image: ROS2 Humble + Nav2 + SLAM + drivers + app gateway deps.
# Built on the local validated nav image (Docker Hub is unreachable from the Jetson);
# packages come from the Tsinghua mirrors.
FROM c21/ros2-humble:nav

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple

RUN sed -i 's#http://ports.ubuntu.com#http://mirrors.tuna.tsinghua.edu.cn#g' /etc/apt/sources.list \
 && rm -f /etc/apt/sources.list.d/*.list \
 && echo "deb [trusted=yes] http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/tuna-ros2.list \
 && apt-get -o Acquire::AllowInsecureRepositories=true update \
 && apt-get install -y --no-install-recommends --allow-unauthenticated \
      ros-humble-orbbec-camera \
      ros-humble-twist-mux \
      ros-humble-imu-filter-madgwick \
      ros-humble-apriltag-ros \
      ros-humble-apriltag-msgs \
      ros-humble-robot-state-publisher \
      ros-humble-xacro \
      ros-humble-tf2-ros \
      ros-humble-tf2-tools \
      ros-humble-nav2-map-server \
      ros-humble-diagnostic-updater \
      ros-humble-image-transport \
      ros-humble-compressed-image-transport \
      ros-humble-rmw-cyclonedds-cpp \
      python3-serial python3-aiohttp python3-yaml python3-pil python3-pip \
      python3-colcon-common-extensions build-essential \
      libzbar0 \
 && rm -rf /var/lib/apt/lists/*

# Keep the distro numpy 1.21 / OpenCV 4.5 that cv_bridge is built against:
# mediapipe is installed without its deps (it would pull numpy 2 + opencv 5).
RUN apt-get update && apt-get install -y --no-install-recommends python3-matplotlib \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install --no-cache-dir pyzbar "numpy<1.22" "protobuf>=4.25,<5" absl-py attrs flatbuffers \
 && pip3 install --no-cache-dir --no-deps mediapipe==0.10.14 \
 && python3 -c "import numpy, cv2; assert numpy.__version__.startswith('1.21'), numpy.__version__; print(cv2.__version__)"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=30

WORKDIR /ws
