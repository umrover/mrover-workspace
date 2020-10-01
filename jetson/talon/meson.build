project('jetson_talon', 'cpp', default_options : ['cpp_std=c++14'])

lcm = dependency('lcm')
phoenix = dependency('phoenix')

all_deps = [lcm, phoenix]

install_headers('rover.hpp')

executable('jetson_talon',
           'main.cpp', 'rover.cpp',
           dependencies : all_deps,
           install : true)
