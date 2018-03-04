project('simulators_perception_lib', 'cpp', version: '1.0.0', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
nanomsg = dependency('nanomsg')

lib = shared_library('percepsim',
                     'percepsim.cpp',
                     dependencies : [opencv, nanomsg],
                     install : true)

install_headers('percepsim.hpp', 'wire_protocol.hpp', subdir : 'percepsim')

pkg = import('pkgconfig')
pkg.generate(name : 'percepsim', description : 'Perception Simulator Library',
             libraries : [lib],
             version : meson.project_version())

threads = dependency('threads')
percepsim_test = executable('percepsim_test', 'percepsim_test.cpp',
                            dependencies : [opencv, nanomsg, threads],
                            link_with : [lib])
test('PercepSim library test', percepsim_test)
