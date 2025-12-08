# Dockerfile per ROS2 Publisher con comunicazione micro-ROS Agent
FROM --platform=linux/arm64 osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Installa dipendenze
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Crea workspace
WORKDIR /root/ros2_ws

# Copia il package my_publisher nel container
COPY ros2_ws/src /root/ros2_ws/src

# Source ROS2 e compila il workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Configura environment per il sourcing automatico
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Configura ROS_DOMAIN_ID per comunicazione tra container (stesso dominio dell'Agent)
ENV ROS_DOMAIN_ID=0

# Configura DDS discovery per comunicazione con micro-ROS Agent
# Usa Fast-DDS (default in ROS2 Humble)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Script di avvio
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/ros2_ws/install/setup.bash\n\
exec "$@"' > /root/entrypoint.sh && \
    chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["/bin/bash"]