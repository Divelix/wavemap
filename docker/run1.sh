docker run -it --rm \
    --name="ros1_catkin" \
    -v="$(pwd)/../:/root/ros_ws/src/wavemap" \
    ros1_catkin:latest