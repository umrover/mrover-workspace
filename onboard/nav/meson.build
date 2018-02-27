project('nav', 'cpp', default_options : ['cpp_std=c++14'])

liblcm = dependency('lcm')

executable('onboard_nav', 'main.cpp', 'layer1.cpp', 'layer2.cpp', 'pid.cpp', 'calculations.cpp',
           dependencies : [liblcm],
           install : true)

layer2_test = executable('layer2_test', 'layer2_test.cpp', 'layer1.cpp', 'layer2.cpp', 'pid.cpp', 'calculations.cpp',
			dependencies: [liblcm])

test('layer2 test', layer2_test)