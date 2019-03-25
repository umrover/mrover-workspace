import venv
import os
import re
import shutil
from contextlib import contextmanager

from invoke.context import Context
from jinja2 import Environment, FileSystemLoader


class BuildError(Exception):
    pass


class WorkspaceContext:
    def __init__(self, root):
        self.root = root
        self.mrover_build_root = os.path.join(os.path.expanduser('~'),
                                              '.mrover')
        self.jarvis_root = os.path.join(root, 'jarvis_files')
        self.third_party_root = os.path.join(root, '3rdparty')
        self.build_intermediate = os.path.join(self.mrover_build_root,
                                               'scratch')
        self.product_env = os.path.join(self.mrover_build_root, 'build_env')
        self.jarvis_env = os.path.join(self.mrover_build_root, 'jarvis_env')
        self.hash_store = os.path.join(self.mrover_build_root,
                                       'project_hashes')

        self.templates = Environment(loader=FileSystemLoader(
            os.path.join(self.jarvis_root, 'templates')))

        self.ctx = Context()

    def ensure_dir(self, d):
        """
        Creates a directory if it does not exist. After invocation of this
        function, you can be ensured the directory exists.

        Parameters:
        d - the path to the directory to create.

        Raises:
        BuildError if there is a file named `d`.
        """
        if not os.path.exists(d):
            os.makedirs(d)
        else:
            if not os.path.isdir(d):
                raise BuildError("{} already exists and is a file".format(d))

    def ensure_build_dirs(self):
        """
        Ensures the build directory structure exists.
        """
        self.ensure_dir(self.mrover_build_root)
        self.ensure_dir(self.hash_store)

    def ensure_product_env(self, clear=False):
        """
        Ensures the product venv existence. If clear is True, re-creates
        the product venv.
        """
        self.ensure_build_dirs()
        if not os.path.isdir(self.product_env) and not clear:
            venv.create(self.product_env, clear=clear,
                        symlinks=True, with_pip=True)

    @contextmanager
    def inside_product_env(self):
        """
        A context manager for activating the product venv.
        """
        with self.ctx.prefix("source {}/bin/activate".format(
                self.product_env)):
            yield

    def template(self, name, **kwargs):
        """
        Templates out a file and returns the rendered copy.
        """
        tpl = self.templates.get_template(name)
        return tpl.render(**kwargs)

    @contextmanager
    def cd(self, *args):
        with self.ctx.cd(*args):
            yield

    def run(self, *args, **kwargs):
        return self.ctx.run(*args, **kwargs)

    def get_product_file(self, *args):
        return os.path.join(self.product_env, *args)

    def get_jarvis_file(self, *args):
        return os.path.join(self.jarvis_env, *args)

    @contextmanager
    def intermediate(self, name):
        """
        Create an intermediate build directory, then change directory to it.
        """
        intermediate = os.path.join(self.build_intermediate, name)
        self.ensure_dir(intermediate)
        if os.listdir(intermediate):
            shutil.rmtree(intermediate)
            self.ensure_dir(intermediate)

        with self.cd(intermediate):
            yield intermediate


class BuildContext:
    def __init__(self, dir_, wksp):
        self.dir_ = dir_
        self.name = dir_.replace('/', '_')
        self.wksp = wksp

    @contextmanager
    def scratch_space(self):
        with self.wksp.intermediate(self.name) as i:
            yield i

    @contextmanager
    def cd(self, *args):
        with self.wksp.cd(*args):
            yield

    def run(self, *args, **kwargs):
        return self.wksp.run(*args, **kwargs)
