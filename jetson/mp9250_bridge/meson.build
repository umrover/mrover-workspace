project('jetson_mp9250_bridge', 'cpp', default_options : ['cpp_std=c++1z'])

lcm = dependency('lcm')

all_deps = [lcm]

src = ['main.cpp']

executable('jetson_mp9250_bridge',
            sources: src,
            dependencies: all_deps,
            install : true)