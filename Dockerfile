FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get -y update && apt install curl software-properties-common -y

# Install.sh equivalent

RUN add-apt-repository universe
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update

# Getting Packages
RUN echo "Installing Deps...:"
RUN apt install -y python3 python-is-python3 git python3-rosdep python3-pip python3-rosinstall-generator python3-vcstool build-essential python3-catkin-tools python3.8-venv

# Poetry
ENV PYTHONFAULTHANDLER=1 \
  PYTHONUNBUFFERED=1 \
  PYTHONHASHSEED=random \
  PIP_NO_CACHE_DIR=off \
  PIP_DISABLE_PIP_VERSION_CHECK=on \
  PIP_DEFAULT_TIMEOUT=100 \
  # Poetry's configuration:
  POETRY_NO_INTERACTION=1 \
  POETRY_VIRTUALENVS_CREATE=false \
  POETRY_CACHE_DIR='/var/cache/pypoetry' \
  POETRY_HOME='/usr/local'
  
RUN curl -sSL https://install.python-poetry.org | python3 -

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init

RUN rosdep update

# Install2.sh equivalent
RUN mkdir -p $HOME/arena_ws
WORKDIR $HOME/arena_ws
RUN echo ""
# clone arena-rosnav
COPY . src/arena/arena-rosnav
RUN until vcs import src < src/arena/arena-rosnav/.repos ; do echo "failed to update, retrying..." ; done
#
RUN echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
#python env init
WORKDIR $HOME/arena_ws/src/arena/arena-rosnav
RUN export PYTHON_KEYRING_BACKEND=keyring.backends.fail.Keyring # resolve faster


RUN poetry export -f requirements.txt --output requirements.txt
RUN pip install -r requirements.txt
# FIXME: Can't open /bin/activate
# RUN . "$(poetry env info -p)/bin/activate"
WORKDIR $HOME/arena_ws
 
# Missing Deps
RUN echo "Installing Missing Deps...:"

RUN apt update && apt install -y libopencv-dev liblua5.2-dev libarmadillo-dev ros-noetic-nlopt liblcm-dev
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
 
# Project Install
RUN echo "Installing Project...:"
WORKDIR $HOME/arena_ws/src
RUN git clone https://github.com/ros/catkin.git
WORKDIR $HOME/arena_ws

RUN source /opt/ros/noetic/setup.bash && catkin build

RUN export ROS_MASTER_URI=http://127.0.0.1:11311/
RUN export ROS_IP=127.0.0.1

RUN echo 'source $HOME/arena_ws/devel/setup.bash' >> "$HOME/.bashrc"

# Install3.sh equivalent
WORKDIR $HOME/arena_ws/src/arena/arena-rosnav
RUN python -m pip install -r requirements.txt

# Insanity has led us here
RUN pip install sismic toml matplotlib numba
RUN apt install -y python3-tk

WORKDIR $HOME/arena_ws

# Install Ros-Bridge
RUN apt install ros-noetic-foxglove-bridge -y

# Fake a display
ENV DEBIAN_FRONTEND noninteractive
ENV DISPLAY :1

RUN apt-get -y install xvfb
 
CMD Xvfb :99 -screen 0 1024x768x24 & export DISPLAY=:99 && source /arena_ws/devel/setup.bash && roslaunch arena_bringup start_arena.launch visualization:=none headless:=2
# CMD python -m pip show sismic