project('percep_obs_detect', 'cuda', 'cpp')

#cuda_comp = meson.get_compiler('cuda')
#cuda = import('unstable-cuda')

#cuda = dependency('cuda', version : '>=10', modules : ['cublas'])
#all_deps = [cuda]

lcm = dependency('lcm')

incdir = include_directories('include/')
#zed_dep = dependency('ZED', method : 'auto')
#all_deps = [zed_dep]

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
all_deps = [zed, lcm]

obs_include_dirs = include_directories('/usr/local/include/pcl-1.11', 
	'/usr/local/include/eigen3', '/usr/include/boost', '/usr/local/include/vtk-8.2', '/usr/include/flann')
	obs = declare_dependency(
	include_directories : obs_include_dirs,
	link_args : [
		# PCL
            '-L/usr/local/lib',
			'-lpcl_common', '-lpcl_visualization', '-lpcl_filters', '-lpcl_segmentation', '-lpcl_search', '-lpcl_kdtree', '-lpcl_sample_consensus',
			'-lpcl_io','-lboost_system','-lvtksys-8.2', '-lvtkRenderingCore-8.2', '-lvtkCommonCore-8.2',
			'-lvtkCommonDataModel-8.2', '-lvtkCommonMath-8.2', '-lvtkFiltersCore-8.2', '-lvtkCommonExecutionModel-8.2', '-lvtkFiltersGeometry-8.2', 
            '-lpthread', '-lGL', '-lglut', '-lGLEW'
	])
	all_deps += [obs]

# Helfupl cuda build flags
debug_args = [ '-res-usage']
speed_args = ['--use_fast_math']

# Not sure if the arch_args help at all. Might be worth investigating
# These are specific to GPU. If you run deviceQuery and see your compute compatibility the major/minor corresponds to the numver listed below
# My compute compatibility for these flags was 7.5
arch_args = ['-arch=compute_75', '-code=sm_75']

exe = executable('percep_obs_detect', 'src/obs-detector.cpp', 'src/common.cu', 'src/euclidean-cluster.cu', 'src/find-clear.cu', 'src/GLViewer.cpp', 
                'src/pass-through.cu', 'src/pcl.cpp', 'src/plane-ransac.cu', 'src/voxel-grid.cu', install : true, dependencies : all_deps, include_directories : incdir,
                cuda_args : speed_args )
#exe = executable('jetson_obs_detector', 'src/driver.cpp' , install : true, dependencies : all_deps, include_directories : incdir)
