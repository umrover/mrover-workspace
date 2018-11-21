import os
import shutil

from . import BuildContext


class ConfigBuilder(BuildContext):
    def __init__(self, dir_, wksp):
        super().__init__(dir_, wksp, ['.json', '.ini'])

    def build(self):
        """
        Copies configuration files into rover config directory.
        """
        if not self.files_changed():
            print("{} unchanged, skipping.".format(self.dir_))
            return

        self.wksp.ensure_product_env()
        full_path = os.path.join(self.wksp.root, self.dir_)

        config_path = os.path.join(self.wksp.product_env, 'config', self.name)
        self.wksp.ensure_dir(config_path)

        if os.path.exists(config_path):
            shutil.rmtree(config_path)
        shutil.copytree(full_path, config_path)

        self.save_hash()
