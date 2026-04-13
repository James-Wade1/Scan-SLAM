FROM osrf/ros:jazzy-desktop
ARG DEBIAN_FRONTEND=noninteractive

# Define a username for the new user
ARG NAME=user

ARG NAME=user
ARG UID=1000

RUN usermod -l $NAME -d /home/$NAME -m ubuntu && \
    groupmod -n $NAME ubuntu && \
    usermod -aG sudo,video,dialout $NAME && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    echo "$NAME:$NAME" | chpasswd

# Set up automatic ROS 2 sourcing and colorized output
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$NAME/.bashrc && \
    echo "source /home/$NAME/ros_ws/install/setup.bash" >> /home/$NAME/.bashrc && \
    echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/$NAME/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/$NAME/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/humble/" >> /home/$NAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$NAME/.bashrc && \
    touch /home/$NAME/.hushlogin

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        curl \
        wget \
        ca-certificates \
        vim \
        nano \
        less \
        grep \
        sudo \
        procps \
        net-tools \
        iputils-ping \
        python3-pygame \
        python3-networkx \
        python3-skimage \
        python3-dill \
        python3-ipykernel \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip \
        python3-setuptools \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        tmux \
        git \
        build-essential \
        cmake \
    && rm -rf /var/lib/apt/lists/*

# For each dependency, document it in docker/DEPENDENCIES.md with author, date, and purpose.
USER root
RUN pip3 install --break-system-packages --no-cache-dir tmuxp numpy pandas matplotlib scipy
ENV PATH="$PATH:/home/$NAME/.local/bin"

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi \
 && rosdep update

 # Fix the mmc_rule sudoers file syntax if it exists. This file is inconsequential if it doesn't exist, but it throws an error about improper syntax if it does.
USER root
RUN if [ -f /etc/sudoers.d/mmc_rule ]; then \
        echo "$NAME ALL=(ALL) NOPASSWD: /bin/ip" > /etc/sudoers.d/mmc_rule; \
    fi

USER $NAME

RUN echo "PS1='\[\033[1;34m\]\u@\h\[\033[0m\]:\[\033[1;37m\]\w\[\033[0m\]\\$ '" >> /home/$NAME/.bashrc

WORKDIR /home/$NAME/ros_ws
CMD [ "tail", "-f", "/dev/null" ]