# Main Dockerfile

FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

# RUN apt-get -y update && apt install -y python3 python-is-python3 git python3-rosdep python3-pip python3-rosinstall python3-rosinstall-generator python3-wstool build-essential libopencv-dev liblua5.2-dev libarmadillo-dev ros-noetic-nlopt liblcm-dev libsdl-dev

# #   Install Poetry
# RUN pip3 install poetry \
#     && pip3 install --upgrade pip

# #   Install PyEnv
# WORKDIR /root/
# RUN git clone --depth=1 https://github.com/pyenv/pyenv.git .pyenv
# ENV PYENV_ROOT="/root/.pyenv"
# ENV PATH="${PYENV_ROOT}/shims:${PYENV_ROOT}/bin:${PATH}"

# RUN echo 'eval "$(pyenv init -)"' >> /root/.bashrc
# RUN sed -Ei -e '/^([^#]|$)/ {a export PYENV_ROOT="$HOME/.pyenv" \nexport PATH="$PYENV_ROOT/bin:$PATH"' -e ':a' -e '$!{n;ba};}' /root/.profile
# RUN echo 'eval "$(pyenv init --path)"' >> /root/.profile

# RUN rosdep update

# RUN mkdir -p /root/src/
# WORKDIR /root/src/

# RUN echo -e "source /opt/ros/noetic/setup.sh" >> /root/.bashrc
# RUN echo -e "source /root/devel/setup.sh" >> /root/.bashrc

# RUN pip install torch rospkg PyYAML filelock scipy PyQT5 empy defusedxml wandb lxml seaborn netifaces

# WORKDIR /root/
# RUN source /root/.bashrc \
#     && source /opt/ros/noetic/setup.sh \
#     && catkin_make

# WORKDIR /root/src/
# RUN git clone https://github.com/Arena-Rosnav/arena-rosnav.git
# WORKDIR /root/src/arena-rosnav/
# RUN rosws update
# RUN poetry run poetry install

# WORKDIR /root/
# RUN source /root/.bashrc \
#     && source /opt/ros/noetic/setup.sh \
#     && source /root/devel/setup.sh && catkin_make

RUN apt-get -y update
RUN apt install curl -y
RUN curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav-webrunner/master/install.sh | bash
RUN curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav-webrunner/master/install2.sh | bash

CMD roslaunch arena_bringup start_arena.launch simulator:=gazebo