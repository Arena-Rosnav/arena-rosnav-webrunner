#!/bin/bash -i

set -e

# Store the current working directory
current_dir="$(pwd)"


# Getting Packages
echo "Installing Deps...:"
sudo apt install -y python3 python-is-python3 git python3-rosdep python3-pip python3-rosinstall-generator python3-vcstool build-essential python3-catkin-tools

# Poetry
echo "Installing Poetry...:"
curl -sSL https://install.python-poetry.org | python3 -
if ! grep -q 'export PATH="$HOME/.local/bin"' ~/.bashrc; then
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
fi


echo "Re-Initializing...:"
sudo rm "$ros_sources_list"
sudo rosdep init

rosdep update

# Return to the original working directory
cd "$current_dir"

echo ""
echo "Now please run the second install script in a NEW terminal."
echo "You NEED to open the new terminal AFTER this script finishes."
echo "You can run the second script with the following command:"
echo ""
echo "curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/master/install2.sh | bash"
