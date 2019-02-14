project('onboard_cv_cvtest', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

with_zed = false 			#get_option('with_zed')
offline_test = true		#get_option('offline_test')
perception_debug = true		#get_option('perception_debug')

conf_data = configuration_data()
conf_data.set10('ZED_SDK_PRESENT', with_zed)
conf_data.set10('OFFLINE_TEST', offline_test)
#conf_data.set10('PERCEPTION_DEBUG', perception_debug)
configure_file(
	input: 'config.h.in',
	output: 'config.h',
	configuration: conf_data)

executable('onboard_cv_cvtest',
		   'automated_test.cpp',
		   dependencies : all_deps,
		   install : true)
