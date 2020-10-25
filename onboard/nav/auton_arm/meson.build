project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('onboard_nav', 'main.cpp', 'rover.cpp', 'auton_armStateMachine.cpp', 
           dependencies : [liblcm],
           install : true)
