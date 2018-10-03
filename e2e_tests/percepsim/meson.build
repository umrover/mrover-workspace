project('e2e_percepsim', 'cpp', default_options : ['cpp_std=c++14'])

opencv = dependency('opencv')
percepsim = dependency('percepsim')
nanomsg = dependency('nanomsg')

executable('e2e_percepsim', 'main.cpp', dependencies : [opencv, percepsim, nanomsg], install : true)
