project('jetson_cv_cvtest', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
lcm = dependency('lcm')

all_deps = [opencv, lcm]

conf_data = configuration_data()
configure_file(
	input: 'config.h.in',
	output: 'config.h',
	configuration: conf_data)

executable('jetson_cv_cvtest',
		   'automated_test.cpp',
		   dependencies : all_deps,
		   install : true)
