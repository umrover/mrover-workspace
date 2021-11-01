import os
import time

from . import BuildContext


class MesonBuilder(BuildContext):
    def __init__(self, dir_, wksp, opts):
        super().__init__(dir_, wksp)
        self.opts = opts

    def build(self):

        self.wksp.ensure_product_env()
        full_dir = os.path.join(self.wksp.root, self.dir_)
        with self.scratch_space() as intermediate:
            print("Compiling C++ project...")
            pkg_cfg_path = self.run(
                    'pkg-config --variable pc_path pkg-config').stdout.strip()
            pkg_cfg_path = '{}:{}:{}'.format(
                    os.path.join(self.wksp.product_env, 'lib', 'x86_64-linux-gnu', 'pkgconfig'),  # noqa XXX
                    os.path.join(self.wksp.product_env, 'lib', 'pkgconfig'),
                    pkg_cfg_path)

            with self.cd(full_dir):
                self.run("PKG_CONFIG_PATH={} meson --prefix={} {}".format(
                    pkg_cfg_path,
                    self.wksp.product_env,
                    intermediate))

            if self.opts is not None:
                time.sleep(1) # XXX config file not properly regenerated without this
                config_string = 'meson configure'
                for opt in self.opts:
                    config_string += ' -D{}'.format(opt)

                self.run(config_string)

            self.run('ninja')

            print('Testing...')
            self.run('LD_LIBRARY_PATH="{}/lib" ninja test'.format(
                self.wksp.product_env))

            print('Installing...')
            self.run('ninja install')

