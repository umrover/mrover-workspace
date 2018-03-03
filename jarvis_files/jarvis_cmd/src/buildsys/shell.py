import os

from . import BuildContext


class ShellBuilder(BuildContext):
    def __init__(self, dir_, wksp):
        super().__init__(dir_, wksp, ['.sh'])

    def build(self):
        if not self.files_changed():
            print("{} unchanged, skipping.".format(self.dir_))
            return

        self.wksp.ensure_product_env()
        full_path = os.path.join(self.wksp.root, self.dir_)
        target = os.path.join(self.wksp.product_env, 'bin', self.name)
        self.run("cp {}/main.sh {}".format(full_path, target))
        os.chmod(target, 0o755)

        self.save_hash()
