project('jetson_circ_auton_arm', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

with_zed = get_option('with_zed')
ar_detection = get_option('ar_detection')
ar_record = get_option('ar_record')
perception_debug = get_option('perception_debug')
write_frame = get_option('write_frame')
data_folder = get_option('data_folder')

# PCL Dependencies:
# https://pcl-tutorials.readthedocs.io/en/master/compiling_pcl_posix.html#compiling-pcl-posix

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

conf_data = configuration_data()
conf_data.set10('AR_DETECTION', ar_detection)
conf_data.set10('AR_RECORD', ar_record)
conf_data.set10('ZED_SDK_PRESENT', with_zed)
conf_data.set10('PERCEPTION_DEBUG', perception_debug)
conf_data.set_quoted('DEFAULT_ONLINE_DATA_FOLDER', data_folder)
configure_file(
	input: 'config.h.in',
	output: 'config.h',
	configuration: conf_data)

executable('jetson_circ_auton_arm',
		   'main.cpp', 'camera.cpp', 'artag_detector.cpp',
		   dependencies : all_deps, cpp_args : '-mavx',
		   install : true)
