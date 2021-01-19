project('ra_kinematics', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('onboard_ra_kinematics', 'main.cpp', 'arm_state.cpp', 'kinematics.cpp', 'utils.cpp',
           dependencies : [liblcm],
           install : true)