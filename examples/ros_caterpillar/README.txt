This a ROS2 comparison for the caterpillar example.

Tried: humble, impossible to compile because of SIP, tried git, impossible to compile.

On Arch Linux: 
- paru -S ros2-arch-deps
needed to clone ros2-iron 
git clone https://aur.archlinux.org/ros2-iron.git

then patch PKGBUILD before colcon build with: 
printf '{ "names": { "tracetools" : { "cmake-args": ["-DTRACETOOLS_TRACEPOINTS_EXCLUDED=ON"]}}}' > colcon.meta

# on armv7 you need to remove the option -mno-omit-leaf-frame-pointer from the C flags in /etc/makepkg.conf

- makepkg -si
On an RPi it will take an eternity to compile.

Then in this directory:
export ROS_DOMAIN_ID=42
source /opt/ros/iron/setup.zsh   # or .bash
colcon build

# then again a this
source install/setup.zsh





