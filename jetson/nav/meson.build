project('nav', 'cpp', default_options : ['cpp_std=gnu++1z'])

liblcm = dependency('lcm')

executable('jetson_nav', 'main.cpp', 'stateMachine.cpp', 'rover.cpp', 'obstacle_avoidance/obstacleAvoidanceStateMachine.cpp', 'obstacle_avoidance/simpleAvoidance.cpp', 'pid.cpp', 'utilities.cpp',
			'search/searchFromPathFile.cpp', 'search/searchStateMachine.cpp',
            'gate_search/gateStateMachine.cpp', 'courseProgress.cpp', 'environment.cpp',
           dependencies : [liblcm],
           install : true)
