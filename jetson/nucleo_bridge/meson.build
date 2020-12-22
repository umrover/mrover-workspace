project('jetson_nucleo_bridge', 'cpp', default_options : ['cpp_std=c++1z'])

lcm = dependency('lcm')
rapidjson = dependency('RapidJSON')

all_deps = [lcm, rapidjson]

install_headers('Controller.h', 'ControllerMap.h', 'I2C.h', 'LCMHandler.h', 'Hardware.h')
src = ['main.cpp', 'ControllerMap.cpp', 'I2C.cpp', 'LCMHandler.cpp', 'Controller.cpp']

executable('jetson_nucleo_bridge',
           sources: src,
           dependencies : all_deps,
           install : true)
