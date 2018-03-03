project('simulators_perception_gazebo', 'cpp', default_options : ['cpp_std=c++14'])

libgazebo = dependency('gazebo', version : '>=8.0.0')
libpercepsim = dependency('percepsim')

shared_library('percepsim_gazebo',
               'gzplugin.cpp',
               install : true)
