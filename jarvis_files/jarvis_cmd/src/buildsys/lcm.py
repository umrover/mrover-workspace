import os

from . import BuildContext
from .python import generate_setup_py, pyinstall


class LCMBuilder(BuildContext):
    def __init__(self, dir_, wksp):
        super().__init__(dir_, wksp, ['.lcm'])

    def build(self):
        """
        Builds an LCM module into Python and C++ modules.
        """
        if not self.files_changed():
            print("{} unchanged, skipping.".format(self.dir_))
            return

        self.wksp.ensure_product_env()
        full_path = os.path.join(self.wksp.root, self.dir_)
        lcm_files = [
                f.path
                for f in os.scandir(full_path)
                if os.path.splitext(f.name)[1] == '.lcm'
        ]
        lcm_files_cmdline = ' '.join('"{}"'.format(v) for v in lcm_files)

        with self.scratch_space() as intermediate:
            print('Generating Python package for LCM project...')
            self.run('lcm-gen --python {} --ppath {}'.format(
                lcm_files_cmdline, intermediate))

            with open(os.path.join(intermediate, 'setup.py'), 'w') as setup_py:
                setup_py.write(generate_setup_py(self, 'rover_msgs',
                                                 src=False))

            with self.wksp.inside_product_env():
                pyinstall(self)

            # TODO build C++ project

        self.save_hash()
