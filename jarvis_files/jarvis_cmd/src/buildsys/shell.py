import os

from . import BuildContext


class ShellBuilder(BuildContext):
    def build(self):

        self.wksp.ensure_product_env()
        full_path = os.path.join(self.wksp.root, self.dir_)
        target = os.path.join(self.wksp.product_env, 'bin', self.name)
        self.run("cp {}/main.sh {}".format(full_path, target))
        os.chmod(target, 0o755)

