# Install dependencies
cd $HOME/ros2_jazzy/
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Go to the git repository
cd $HOME/ros2_jazzy/src/px4_portfolio

# Copy extra files to the PX4 folder
cp -vr missions/extras/models/* $HOME/PX4-Autopilot/Tools/simulation/gz/models
cp -vr missions/extras/worlds/* $HOME/PX4-Autopilot/Tools/simulation/gz/worlds
cp -vr missions/extras/airframes/* $HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes

# Rebuild PX4
cd $HOME/PX4-Autopilot
make px4_sitl gz_x500_custom_2


