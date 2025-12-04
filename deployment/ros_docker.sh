#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="ros2-jazzy-demo"
CONTAINER_NAME="ros2-jazzy-dev"
ROS_DISTRO="jazzy"
WORKSPACE_DIR="$PWD"

usage() {
  echo "Usage: $0 {build|shell|run-talker|run-listener}"
  echo
  echo "  build        Build the Docker image"
  echo "  shell        Start an interactive shell in the container (ROS sourced)"
  echo "  run-talker   Run ROS 2 C++ demo talker"
  echo "  run-listener Run ROS 2 C++ demo listener"
}

cmd_build() {
  docker build -t "${IMAGE_NAME}" .
}

cmd_shell() {
  docker run --rm -it \
    --net=host \
    --name "${CONTAINER_NAME}" \
    -v "${WORKSPACE_DIR}/src":/ros2_ws/src \
    "${IMAGE_NAME}" \
    bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
              source /ros2_ws/install/setup.bash 2>/dev/null || true; \
              exec bash"
}

cmd_run_talker() {
  docker run --rm -it \
    --net=host \
    --name "${CONTAINER_NAME}-talker" \
    "${IMAGE_NAME}" \
    bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
              ros2 run demo_nodes_cpp talker"
}

cmd_run_listener() {
  docker run --rm -it \
    --net=host \
    --name "${CONTAINER_NAME}-listener" \
    "${IMAGE_NAME}" \
    bash -lc "source /opt/ros/${ROS_DISTRO}/setup.bash && \
              ros2 run demo_nodes_cpp listener"
}

main() {
  case "${1:-}" in
    build)        cmd_build ;;
    shell)        cmd_shell ;;
    run-talker)   cmd_run_talker ;;
    run-listener) cmd_run_listener ;;
    *)            usage; exit 1 ;;
  esac
}

main "$@"