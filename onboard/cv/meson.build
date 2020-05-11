project('onboard_cv', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

with_zed = get_option('with_zed')

if with_zed
	zed_include_dirs = include_directories('/usr/local/zed/include', '/usr/local/cuda/include')
	zed = declare_dependency(
		include_directories : zed_include_dirs,
		link_args : [
			# ZED SDK
			'-L/usr/local/zed/lib',
			'-lsl_zed',

			# CUDA 
			'-L/usr/local/cuda/lib64',
			'-lcuda', '-lnppial', '-lnppisu', '-lnppicc',
			'-lnppicom', '-lnppidei', '-lnppif', '-lnppig',
			'-lnppim', '-lnppist', '-lnppitc', '-lnppc'
		])
	all_deps += [zed]
endif

ar_detection = get_option('ar_detection')
obs_detection = get_option('obs_detection')
perception_debug = get_option('perception_debug')
write_frame = get_option('write_frame')
data_folder = get_option('data_folder')
obs_record = get_option('obs_record')
ar_record = get_option('ar_record')

conf_data = configuration_data()
conf_data.set10('AR_DETECTION', ar_detection)
conf_data.set10('OBSTACLE_DETECTION', obs_detection)
conf_data.set10('ZED_SDK_PRESENT', with_zed)
conf_data.set10('PERCEPTION_DEBUG', perception_debug)
conf_data.set10('WRITE_CURR_FRAME_TO_DISK', write_frame)
conf_data.set10('OBS_RECORD', obs_record)
conf_data.set10('AR_RECORD', ar_record)
conf_data.set_quoted('DEFAULT_ONLINE_DATA_FOLDER', data_folder)
configure_file(
	input: 'config.h.in',
	output: 'config.h',
	configuration: conf_data)

executable('onboard_cv',
		   'main.cpp', 'camera.cpp', 'artag_detector.cpp', 'obstacle_detector.cpp',
		   dependencies : all_deps,
		   install : true)
