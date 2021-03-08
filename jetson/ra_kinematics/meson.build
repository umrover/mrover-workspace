project('ra_kinematics', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('jetson_ra_kinematics', 'main.cpp', 'arm_state.cpp', 'kinematics.cpp', 'utils.cpp', 'mrover_arm.cpp', 'motion_planner.cpp', 'spline.cpp',
           dependencies : [liblcm],
           install : true)