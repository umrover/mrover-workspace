project('onboard_cv', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

# TODO Jarvis will need to be smartened up to build this properly
with_zed = get_option('with_zed')
if with_zed
	zed_include_dirs = include_directories('/usr/local/zed/include')
	zed = declare_dependency(
		include_directories : zed_include_dirs,
		link_args : [
			# ZED SDK
			'-L/usr/local/zed/lib',
			'-lsl_zed', '-lsl_depthcore', '-lsl_calibration',
			'-lsl_tracking', '-lsl_disparityFusion', '-lsl_svorw',
			'-lsl_scanning', '-lsl_core',

			# CUDA 
			'-L/usr/local/cuda/lib64',
			'-lnppial', '-lnppisu', '-lnppicc', '-lnppicom',
			'-lnppidei', '-lnppif', '-lnppig', '-lnppim',
			'-lnppist', '-lnppitc'
		])
	all_deps += [zed]
else
	percepsim = dependency('percepsim')
	nanomsg = dependency('nanomsg')
	all_deps += [percepsim, nanomsg]
endif

conf_data = configuration_data()
conf_data.set10('ZED_SDK_PRESENT', with_zed)
configure_file(
	input: 'config.h.in',
	output: 'config.h',
	configuration: conf_data)

executable('onboard_cv',
		   'main.cpp', 'camera.cpp',
		   dependencies : all_deps,
		   install : true)
