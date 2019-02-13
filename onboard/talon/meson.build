project('onboard_talon', 'cpp', default_options : ['cpp_std=c++14'])

lcm = dependency('lcm')
phoenix = dependency('phoenix')

all_deps = [lcm, phoenix]

executable('onboard_talon',
           'main.cpp',
           dependencies : all_deps,
           install : true)
