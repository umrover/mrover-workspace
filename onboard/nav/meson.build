project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('onboard_nav', 'main.cpp', 'layers.cpp', 'layer1.cpp', 'pid.cpp',
           dependencies : [liblcm],
           install : true)
