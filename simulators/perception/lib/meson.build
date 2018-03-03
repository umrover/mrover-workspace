project('simulators_perception_lib', 'cpp', version: '1.0.0', default_options : ['cpp_std=c++14'])

shared_library('percepsim',
               'percepsim.cpp',
               install : true)

install_headers('percepsim.hpp', subdir : 'percepsim')

pkg = import('pkgconfig')
pkg.generate(name : 'percepsim', description : 'Perception Simulator Library',
             version : meson.project_version())
