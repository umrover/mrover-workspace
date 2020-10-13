project('jetson_cv', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

with_zed = true

if with_zed
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
endif

conf_data = configuration_data()
conf_data.set10('ZED_SDK_PRESENT', with_zed)
configure_file(
	input: 'config.h.in',
	output: 'config.h',
	configuration: conf_data)

executable('jetson_cv_calibrate',
		   'main.cpp', 'camera.cpp',
		   dependencies : all_deps,
		   install : true)
