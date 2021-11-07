import os

from . import BuildContext


class CMakeBuilder(BuildContext):
    def __init__(self, dir_, wksp, opts):
        super().__init__(dir_, wksp)
        self.opts = opts

    def build(self):

        self.wksp.ensure_product_env()
        full_dir = os.path.join(self.wksp.root, self.dir_)
        with self.scratch_space() as intermediate:
            print("Compiling CMake project...")
            pkg_cfg_path = self.run(
                    'pkg-config --variable pc_path pkg-config').stdout.strip()
            pkg_cfg_path = '{}:{}:{}'.format(
                    os.path.join(self.wksp.product_env, 'lib', 'x86_64-linux-gnu', 'pkgconfig'),  # noqa XXX
                    os.path.join(self.wksp.product_env, 'lib', 'pkgconfig'),
                    pkg_cfg_path)

            cmake_buildir = "build"
            cmake_build_path = os.path.join(intermediate, cmake_buildir)

            os.mkdir(cmake_build_path)
            with self.cd(full_dir):
                self.run("cmake -B{} -H{} -DCMAKE_PREFIX_PATH={}".format(
                    cmake_build_path,
                    full_dir,
                    pkg_cfg_path))

            with self.cd(cmake_build_path):
                self.run("make all")
