project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('jetson_nav_auton_arm', 'main.cpp', 'rover.cpp', 'stateMachine.cpp', 
           dependencies : [liblcm],
           install : true)
