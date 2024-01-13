FROM ros:humble

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
        g++ \
        cmake \
        git \
        libsocketcan-dev \
        can-utils \
        libeigen3-dev \
        gdb

ADD https://api.github.com/repos/JTylerBoylan/starq-lib/git/refs/heads/main version.json
RUN git clone https://github.com/JTylerBoylan/starq-lib
RUN cd starq-lib && mkdir build
RUN cd starq-lib/build && cmake ..
RUN cd starq-lib/build && cmake --build . --target install
RUN ldconfig

# Create a new user with a specific UID and GID, and set up the workspace
RUN useradd -m -u 1000 -s /bin/bash user && \
    mkdir -p /ros2_ws && \
    chown -R user:user /ros2_ws
WORKDIR /ros2_ws/src/

# Switch to the new non-root user
USER user

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the default command to execute when creating a new container
CMD ["bash"]
