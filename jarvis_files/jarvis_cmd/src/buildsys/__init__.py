import venv
import os
import re
import shutil
import hashlib
from contextlib import contextmanager

from invoke.context import Context
from jinja2 import Environment, FileSystemLoader


def hash_file(filepath, algo):
    hasher = algo()
    blocksize = 64 * 1024  # TODO see if there is a more efficient way
    with open(filepath, 'rb') as fp:
        while True:
            data = fp.read(blocksize)
            if not data:
                break
            hasher.update(data)

    return hasher.hexdigest()


def hash_dir(dir_, suffixes):
    """
    Computes the SHA-256 hash of all files with the given suffixes.
    """
    hash_algo = hashlib.sha256
    if not os.path.isdir(dir_):
        raise BuildError('{} is not a directory.'.format(dir_))
    intermediate_hashes = []
    for root, dirs, files in os.walk(dir_, topdown=True, followlinks=False):
        if not re.search(r'/\.', root):
            intermediate_hashes.extend(
                [
                    hash_file(os.path.join(root, f), hash_algo)
                    for f in files
                    if not f.startswith('.') and not re.search(r'/\.', f)
                    and os.path.splitext(f)[1] in suffixes
                ]
            )

    hasher = hash_algo()
    for hashvalue in sorted(intermediate_hashes):
        hasher.update(hashvalue.encode('utf-8'))
    return hasher.hexdigest()


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
    def intermediate(self, name, cleanup=False):
        """
        Create an intermediate build directory, then change directory to it.
        """
        intermediate = os.path.join(self.build_intermediate, name)
        self.ensure_dir(intermediate)
        if os.listdir(intermediate) and cleanup:
            shutil.rmtree(intermediate)
            self.ensure_dir(intermediate)

        with self.cd(intermediate):
            yield intermediate

        if cleanup:
            shutil.rmtree(intermediate)


class BuildContext:
    def __init__(self, dir_, wksp, suffixes):
        self.dir_ = dir_
        self.name = dir_.replace('/', '_')
        self.wksp = wksp
        self.suffixes = suffixes

    @contextmanager
    def scratch_space(self, cleanup=False):
        with self.wksp.intermediate(self.name, cleanup) as i:
            yield i

    def files_changed(self):
        """
        Determines whether the directory should be re-built.
        """
        hash_file_path = os.path.join(self.wksp.hash_store, self.name)
        saved_hash = b''
        try:
            with open(hash_file_path) as hash_file:
                saved_hash = hash_file.read()
        except:
            pass

        full_path = os.path.join(self.wksp.root, self.dir_)
        computed_hash = hash_dir(full_path, self.suffixes)

        return saved_hash != computed_hash

    def save_hash(self):
        """
        Saves the hash of the build directory.
        """
        full_path = os.path.join(self.wksp.root, self.dir_)
        hash_file_path = os.path.join(self.wksp.hash_store, self.name)
        with open(hash_file_path, 'w') as hash_file:
            hash_file.write(hash_dir(full_path, self.suffixes))

    @contextmanager
    def cd(self, *args):
        with self.wksp.cd(*args):
            yield

    def run(self, *args, **kwargs):
        return self.wksp.run(*args, **kwargs)
