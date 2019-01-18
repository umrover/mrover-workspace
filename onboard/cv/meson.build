project('onboard_cv', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

zed_include_dirs = include_directories('/usr/local/zed/include', '/usr/local/cuda/include')
zed = declare_dependency(
	include_directories : zed_include_dirs,
	link_args : [
		# ZED SDK
		'-L/usr/local/zed/lib',
		'-lsl_input', '-lsl_core', '-lsl_zed',

		# CUDA 
		'-L/usr/local/cuda/lib64',
		'-lnppial', '-lnppisu', '-lnppicc', '-lnppicom',
		'-lnppidei', '-lnppif', '-lnppig', '-lnppim',
		'-lnppist', '-lnppitc'
	])
all_deps += [zed]

executable('onboard_cv',
		   'main.cpp', 'camera.cpp', 'tennisball_detector.cpp', 'obstacle_detector.cpp',
		   dependencies : all_deps,
		   install : true)
