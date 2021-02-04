project('ra_kinematics', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('jetson_ra_kinematics', 'test/kinematics_tests.cpp', 'test/arm_state_tests.cpp', 'arm_state.cpp', 'kinematics.cpp', 'utils.cpp',
           dependencies : [liblcm],
           install : true)