project('base_station', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('base_station_test', 'main.cpp',
           dependencies : [liblcm],
           install : true)