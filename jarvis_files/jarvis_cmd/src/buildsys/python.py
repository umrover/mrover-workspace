import os
import shutil

from invoke.exceptions import UnexpectedExit
from . import BuildContext


def _handle_pytest_exits(ctx, command):
    try:
        ctx.run(command)
    except UnexpectedExit as e:
        if e.result.exited != 5:
            raise e


def generate_setup_py(ctx, component_name, *, executable=False, src=True):
    """
    Generates a setup.py file for a component.
    """
    return ctx.wksp.template('setup.py', component=component_name,
                             executable=executable, src=src)


def pytest(ctx, doctests=True):
    """
    Runs pytest on a project, handling the case where no tests are run.
    Assumes the context is currently inside an intermediate build directory.
    """
    if doctests:
        print("Running doctests...")
        with ctx.cd('src'):
            _handle_pytest_exits(ctx, "pytest --doctest-modules")

    print("Running pytests...")
    _handle_pytest_exits(ctx, "pytest")


def pylint(ctx):
    """
    Lints python code with flake8.
    """
    print("Linting...")
    ctx.run("flake8 --max-line-length=120")


def pyinstall(ctx):
    """
    Installs a Python module into the current env.
    """
    print("Installing...")
    ctx.run("python setup.py install")


class PythonBuilder(BuildContext):
    def __init__(self, dir_, wksp, lint, executable):
        super().__init__(dir_, wksp)
        self.lint = lint
        self.executable = executable

    def build(self):
        """
        Builds a Python module.
        """

        self.wksp.ensure_product_env()
        full_path = os.path.join(self.wksp.root, self.dir_)
        with self.scratch_space() as intermediate:
            srcdir = os.path.join(full_path, 'src')
            build_srcdir = os.path.join(intermediate, 'src')
            build_pkgdir = os.path.join(build_srcdir, self.name)
            if os.path.exists(build_pkgdir):
                shutil.rmtree(build_pkgdir)
            self.wksp.ensure_dir(build_srcdir)
            shutil.copytree(srcdir, build_pkgdir)

            # Generate an __init__.py file
            self.run("touch {}".format(
                os.path.join(build_pkgdir, '__init__.py')))

            # Generate a setup.py file
            with open(os.path.join(intermediate, 'setup.py'), 'w') as setup_py:
                setup_py.write(
                        generate_setup_py(self, self.name,
                                          executable=self.executable))

            if self.lint:
                pylint(self)

            with self.wksp.inside_product_env():
                pytest(self)
                pyinstall(self)

