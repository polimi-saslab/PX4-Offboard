# Colcon build
alias cb='colcon build'
alias cb_pkg='colcon build --packages-select'
alias cb_seq='colcon build --executor sequential'
alias cb_skip='colcon build --packages-skip'
alias cb_unbuilt='colcon build --packages-skip-build-finished'
# colcon build arguments
alias cb_verbose='--event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON'

# ROS2
alias tolist='ros2 topic list'
alias toecho='tos2 topic echo'
alias wsin='. install/setup.bash'

# HITL ROS2
alias hlaunch='ros2 launch px4_ros_bridge'

# Directory navigation
# List by size
alias lt='ls --human-readable --size -1 -S --classify'