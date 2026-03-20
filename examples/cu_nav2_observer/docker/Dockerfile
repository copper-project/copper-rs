FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/copper_nav2

COPY docker/auto_nav_demo.py /opt/copper_nav2/auto_nav_demo.py
COPY docker/run_nav2_demo_inside_container.sh /opt/copper_nav2/run_nav2_demo_inside_container.sh

RUN chmod +x /opt/copper_nav2/run_nav2_demo_inside_container.sh

CMD ["/bin/bash", "-lc", "/opt/copper_nav2/run_nav2_demo_inside_container.sh"]
