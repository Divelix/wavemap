docker run -it --rm \
    --name="ros2_colcon" \
    -v "$(pwd)/../:/root/ros_ws/src/wavemap" \
    ros2_colcon:latest