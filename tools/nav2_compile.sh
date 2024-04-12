#!/bin/bash
export CC=/usr/bin/clang
export CXX=/usr/bin/clang++
export MAKEFLAGS="-j4"

source /opt/ros/humble/setup.bash
cd $HOME/src/nav2_240315
# source ...nav2_home

# check dependencies
rosdep install -y --from-paths ./src --ignore-src

colcon build \
	--symlink-install \
	--parallel-workers 2 \
	--cmake-args \
		-DBUILD_TESTING=OFF \
		-DCMAKE_CXX_STANDARD=17 \
		-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
		-DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -w -Wno-error -Wno-format-securtiy" \
		-DCMAKE_C_FLAGS="${CMAKE_C_FLAGS} -w -Wno-error -Wno-format-security"

# use asan and coverage
pkgs_name="nav2_amcl nav2_behaviors nav2_bringup nav2_bt_navigator nav2_collision_monitor nav2_common nav2_constrained_smoother nav2_controller nav2_core nav2_dwb_controller nav2_map_server nav2_mppi_controller nav2_navfn_planner nav2_planner nav2_regulated_pure_pursuit_controller nav2_rotation_shim_controller nav2_simple_commander nav2_smac_planner nav2_smoother nav2_theta_star_planner nav2_velocity_smoother"

colcon build \
    --cmake-clean-cache \
    --packages-select $pkgs_name \
    --cmake-args \
        -DBUILD_TESTING=OFF \
        -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -w -Wno-error -Wno-everything -fsanitize=address --coverage -DCOVERAGE_RUN=1"  \
        -DCMAKE_C_FLAGS="${CMAKE_C_FLAGS} -w -Wno-error -Wno-format-security -fsanitize=address --coverage -DCOVERAGE_RUN=1"
