import os
import shutil

from . import BuildContext


def generate_webapp_start(ctx, app_dir, port):
    return ctx.wksp.template('webapp_start', app_dir=app_dir, port=port)


class RollupJSBuilder(BuildContext):
    def __init__(self, dir_, wksp, deps, app=True, port=8010):
        super().__init__(dir_, wksp)
        self.deps = [n.replace('/', '_') for n in deps]
        self.app = app
        self.port = port

    def build(self):

        self.wksp.ensure_product_env()
        full_path = os.path.join(self.wksp.root, self.dir_)
        with self.scratch_space() as intermediate:
            self.run("cp -r {}/* .".format(full_path))

            if self.app:
                print('Building JS project...')
                self.run("yarn")
                if not os.path.exists(os.path.join(intermediate, 'deps')):
                    os.mkdir(os.path.join(intermediate, 'deps'))

                for dep in self.deps:
                    dep_module_dir = os.path.join(
                            self.wksp.product_env, 'share', 'js', dep)
                    dep_dir = os.path.join(
                            intermediate, 'deps', dep)
                    if os.path.exists(dep_dir):
                        os.unlink(dep_dir)
                    os.symlink(dep_module_dir, dep_dir)
                self.run("yarn run build")

            # copy distribution files and package.json into product env
            dist_dir = os.path.join(self.wksp.product_env, 'share', 'js',
                                    self.name)
            self.wksp.ensure_dir(dist_dir)
            if os.path.exists(os.path.join(dist_dir, 'dist')):
                shutil.rmtree(os.path.join(dist_dir, 'dist'))
            shutil.copytree(os.path.join(intermediate, 'dist'),
                            os.path.join(dist_dir, 'dist'))
            if os.path.exists(os.path.join(intermediate, 'package.json')):
                shutil.copyfile(os.path.join(intermediate, 'package.json'),
                                os.path.join(dist_dir, 'package.json'))

            if os.path.exists(os.path.join(intermediate, 'yarn.lock')):
                shutil.copyfile(os.path.join(intermediate, 'yarn.lock'),
                                os.path.join(dist_dir, 'yarn.lock'))

            if self.app:
                script_path = os.path.join(
                        self.wksp.product_env, 'bin', self.name)
                with open(script_path, 'w') as start_script:
                    start_script.write(
                            generate_webapp_start(self, dist_dir, self.port))
                os.chmod(script_path, 0o755)

            print('Build successful.')
