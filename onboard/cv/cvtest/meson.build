project('onboard_cv_cvtest', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

zed_include_dirs = include_directories('/usr/local/zed/include', '/usr/local/cuda/include')
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

executable('onboard_cv_cvtest',
		   'automated_test.cpp',
		   dependencies : all_deps,
		   install : true)
