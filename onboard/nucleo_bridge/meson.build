project('onboard_nucleo_bridge', 'cpp', default_options : ['cpp_std=c++1z'])

lcm = dependency('lcm')

all_deps = [lcm]

install_headers('backend.h','frontend.h','hardware.h','controller.h')

executable('onboard_nucleo_bridge',
           'main.cpp',
           dependencies : all_deps,
           install : true)
