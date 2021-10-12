import os
import time

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
            cmake_buildir = "build"
            cmake_build_path = os.path.join(intermediate, cmake_buildir)

            os.mkdir(cmake_build_path)
            with self.cd(full_dir):
                self.run("cmake -B{} -H{}".format(
                    cmake_build_path,
                    full_dir))