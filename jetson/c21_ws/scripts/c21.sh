#!/usr/bin/env bash
# C21 runtime helper (run on the Jetson host).
#   c21.sh build        colcon build the workspace inside the runtime image
#   c21.sh up [args]    start the robot stack container (robot.launch.py args)
#   c21.sh down         stop it (driver sends a stop burst on SIGINT)
#   c21.sh logs | shell | ros2 <args...>
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${C21_IMAGE:-c21/runtime:p1}"
NAME=c21_robot
RUN_OPTS=(--net=host --ipc=host --pid=host --privileged -v /dev:/dev -v "$WS":/ws
          -v "$HOME/.ros":/root/.ros -v "$HOME/c21_maps":/root/c21_maps
          -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -w /ws)
SETUP='source /opt/ros/humble/setup.bash; [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash'
mkdir -p "$HOME/.ros" "$HOME/c21_maps"

case "${1:-}" in
  build)
    shift
    docker run --rm "${RUN_OPTS[@]}" "$IMAGE" bash -c \
      "source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release $*" ;;
  up)
    shift
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d --name "$NAME" --restart always --stop-signal SIGINT --stop-timeout 15 \
      "${RUN_OPTS[@]}" "$IMAGE" bash -c "$SETUP; exec ros2 launch c21_bringup robot.launch.py $*"
    echo "started $NAME" ;;
  down) docker stop "$NAME" && docker rm "$NAME" ;;
  logs) shift; docker logs --tail "${1:-100}" -f "$NAME" ;;
  shell) docker exec -it "$NAME" bash -c "$SETUP; exec bash" ;;
  ros2) shift; docker exec "$NAME" bash -c "$SETUP; ros2 $(printf '%q ' "$@")" ;;
  test)
    docker run --rm "${RUN_OPTS[@]}" "$IMAGE" bash -c \
      "$SETUP; cd /ws && python3 -m pytest -q src/c21_base/test src/c21_safety/test" ;;
  *) sed -n '2,7p' "$0"; exit 1 ;;
esac
