import os
import time

from . import BuildContext


class CMakeBuilder(BuildContext):
    def __init__(self, dir_, wksp, opts):
        super().__init__(dir_, wksp)
        self.opts = opts

    def build(self):
        print("Compiling CMake project...")