project('jetson_percep', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

with_zed = get_option('with_zed')
obs_detection = get_option('obs_detection')
vm_config = get_option('vm_config')

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

if obs_detection and not vm_config

	obs_include_dirs = include_directories('/usr/local/include/pcl-1.11', 
	'/usr/local/include/eigen3', '/usr/include/boost', '/usr/local/include/vtk-8.2', '/usr/include/flann')
	obs = declare_dependency(
	include_directories : obs_include_dirs,
	link_args : [
		# PCL
			'-lpcl_common', '-lpcl_visualization', '-lpcl_filters', '-lpcl_segmentation', '-lpcl_search', '-lpcl_kdtree', '-lpcl_sample_consensus',
			'-lpcl_io','-lboost_system','-lvtksys-8.2', '-lvtkRenderingCore-8.2', '-lvtkCommonCore-8.2',
			'-lvtkCommonDataModel-8.2', '-lvtkCommonMath-8.2', '-lvtkFiltersCore-8.2', '-lvtkCommonExecutionModel-8.2', '-lvtkFiltersGeometry-8.2'
	])
	all_deps += [obs]

elif obs_detection and vm_config

	obs_include_dirs = include_directories('/usr/local/include/pcl-1.11', 
	'/usr/local/include/eigen3', '/usr/include/boost', '/usr/include/vtk-6.3', '/usr/include/flann')
	obs = declare_dependency(
	include_directories : obs_include_dirs,
	link_args : [
		# PCL
			'-lpcl_common', '-lpcl_visualization', '-lpcl_filters', '-lpcl_segmentation', '-lpcl_search', '-lpcl_kdtree', '-lpcl_sample_consensus',
			'-lpcl_io','-lboost_system','-lvtksys-6.3', '-lvtkRenderingCore-6.3', '-lvtkCommonCore-6.3',
			'-lvtkCommonDataModel-6.3', '-lvtkCommonMath-6.3', '-lvtkFiltersCore-6.3', '-lvtkCommonExecutionModel-6.3', '-lvtkFiltersGeometry-6.3'
	])
	all_deps += [obs]
endif

ar_detection = get_option('ar_detection')
ar_record = get_option('ar_record')
obs_record = get_option('obs_record')
perception_debug = get_option('perception_debug')
write_frame = get_option('write_frame')
data_folder = get_option('data_folder')

conf_data = configuration_data()
conf_data.set10('AR_DETECTION', ar_detection)
conf_data.set10('AR_RECORD', ar_record)
conf_data.set10('OBSTACLE_DETECTION', obs_detection)
conf_data.set10('OBSTACLE_RECORD', obs_record)
conf_data.set10('ZED_SDK_PRESENT', with_zed)
conf_data.set10('PERCEPTION_DEBUG', perception_debug)
conf_data.set10('WRITE_CURR_FRAME_TO_DISK', write_frame)
conf_data.set10('VIRTUAL_MACHINE_CONFIG', vm_config)
conf_data.set_quoted('DEFAULT_ONLINE_DATA_FOLDER', data_folder)
configure_file(
	input: 'config.h.in',
	output: 'config.h',
	configuration: conf_data)

executable('jetson_percep',
		   'main.cpp', 'camera.cpp', 'artag_detector.cpp', 'pcl.cpp',
		   dependencies : all_deps, cpp_args : '-mavx',
		   install : true)
