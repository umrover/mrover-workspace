project('simulators_perception_gazebo', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
gazebo = dependency('gazebo', version : '>=8.0.0')
percepsim = dependency('percepsim')
nanomsg = dependency('nanomsg')
sdformat = dependency('sdformat')
ogre = dependency('OGRE')
ogre_paging = dependency('OGRE-Paging')

depth_camera_plugin = declare_dependency(link_args : ['-lDepthCameraPlugin'])

shared_library('percepsim_gazebo',
               'gzplugin.cpp',
               dependencies : [gazebo, percepsim, sdformat, ogre, ogre_paging, opencv, depth_camera_plugin, nanomsg],
               install : true)
