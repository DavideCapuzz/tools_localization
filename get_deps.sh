set -euo pipefail

# --- detect Ubuntu release and choose ROS distro ---
UBU_VER="$(. /etc/os-release && echo "$VERSION_ID")"
case "$UBU_VER" in
  22.04) ROS_DISTRO=humble ;;
  24.04) ROS_DISTRO=jazzy  ;;
  *) echo "Unsupported Ubuntu $UBU_VER for this script. Adjust ROS_DISTRO manually."; exit 1 ;;
esac
echo "Ubuntu $UBU_VER detected -> ROS 2 $ROS_DISTRO"

# --- base tools ---
sudo apt update
sudo apt install -y curl gnupg lsb-release build-essential git cmake python3-pip python3-venv

# --- ROS 2 apt repo & key ---
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# --- ROS 2 Desktop (brings rclcpp, tf2 stacks, messages, etc.) ---
sudo apt install -y "ros-$ROS_DISTRO-desktop"

# --- colcon & dev tools ---
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

# --- initialize rosdep (once per machine) ---
sudo rosdep init || true
rosdep update

# --- add ROS to shell startup (bash) ---
if ! grep -q "/opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi

# --- workspace skeleton ---
WS=~/ros2_ws
mkdir -p "$WS/src"

echo "Bootstrap done. Open a NEW shell or 'source ~/.bashrc' before building."
