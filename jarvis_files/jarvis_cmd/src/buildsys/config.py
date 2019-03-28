import os
import shutil

from . import BuildContext


class ConfigBuilder(BuildContext):
    def build(self):
        """
        Copies configuration files into rover config directory.
        """

        self.wksp.ensure_product_env()
        full_path = os.path.join(self.wksp.root, self.dir_)

        config_path = os.path.join(self.wksp.product_env, 'config', self.name)
        self.wksp.ensure_dir(config_path)

        if os.path.exists(config_path):
            shutil.rmtree(config_path)
        shutil.copytree(full_path, config_path)

