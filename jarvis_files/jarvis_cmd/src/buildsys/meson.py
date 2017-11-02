import os

from . import BuildContext


class MesonBuilder(BuildContext):
    def __init__(self, dir_, wksp):
        super().__init__(dir_, wksp, ['.h', '.hpp', '.cpp'])

    def build(self):
        if not self.files_changed():
            print("{} unchanged, skipping.".format(self.dir_))
            return

        self.wksp.ensure_product_env()
        full_dir = os.path.join(self.wksp.root, self.dir_)
        with self.scratch_space() as intermediate:
            print("Compiling C++ project...")
            pkg_cfg_path = self.run(
                    'pkg-config --variable pc_path pkg-config').stdout.strip()
            pkg_cfg_path = '{}:{}'.format(
                    os.path.join(self.wksp.product_env, 'lib', 'pkgconfig'),
                    pkg_cfg_path)

            with self.cd(full_dir):
                self.run("PKG_CONFIG_PATH={} meson --prefix={} {}".format(
                    pkg_cfg_path,
                    self.wksp.product_env,
                    intermediate))

            self.run('ninja')

            print('Testing...')
            self.run('ninja test')

            print('Installing...')
            self.run('ninja install')

        self.save_hash()
