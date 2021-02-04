project('ra_kinematics', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('jetson_ra_kinematics', 'test/motion_planner_tests.cpp', 'arm_state.cpp', 'kinematics.cpp', 'motion_planner.cpp', 'utils.cpp',
           dependencies : [liblcm],
           install : true)