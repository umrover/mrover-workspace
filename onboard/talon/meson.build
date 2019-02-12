project('onboard_talon', 'cpp', default_options : ['cpp_std=c++14'])

lcm = dependency('lcm')

talon = declare_dependency(
    include_directories : include_directories('/usr/local/talon/include'),
    link_args : [
        '-L/usr/local/talon/lib',
        '-lCTRE_Phoenix', '-lCTRE_PhoenixCanutils', '-lCTRE_PhoenixCCI', '-lCTRE_PhoenixPlatform_socketcan',
    ])

all_deps = [lcm, talon]

executable('onboard_talon',
           'main.cpp',
           dependencies : all_deps,
           install : true)
