import venv
import os
import re
import shutil
import hashlib
from contextlib import contextmanager

from invoke.exceptions import UnexpectedExit
from jinja2 import Environment, FileSystemLoader

import config


_template_env = Environment(
        loader=FileSystemLoader(
            os.path.join(config.JARVIS_ROOT, 'templates')
        )
    )


class BuildError(Exception):
    pass


def ensure_dir(d):
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


def ensure_build_dirs():
    """
    Ensures the build directory structure exists.
    """
    ensure_dir(config.BUILD_ROOT)
    ensure_dir(config.HASH_STORE)


def ensure_product_env(clear=False):
    """
    Ensures the prodcut venv existence. If clear is True, re-creates
    the product venv.
    """
    ensure_build_dirs()
    if not os.path.isdir(config.PRODUCT_ENV) and not clear:
        venv.create(config.PRODUCT_ENV, clear=clear,
                    symlinks=True, with_pip=True)


def get_product_file(*args):
    """
    Gets a file in the product venv.
    """
    return os.path.join(config.PRODUCT_ENV, *args)


def get_product_executable(name):
    """
    Gets an executable file from the product venv.
    """
    return get_product_file('bin', name)


def get_jarvis_executable(name):
    """
    Gets an executable file from the Jarvis venv.
    """
    return os.path.join(config.JARVIS_ENV, 'bin', name)


def _hash_file(filepath, algo):
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
                    _hash_file(os.path.join(root, f), hash_algo)
                    for f in files
                    if not f.startswith('.') and not re.search(r'/\.', f)
                    and os.path.splitext(f)[1] in suffixes
                ]
            )

    hasher = hash_algo()
    for hashvalue in sorted(intermediate_hashes):
        hasher.update(hashvalue.encode('utf-8'))
    return hasher.hexdigest()


def files_changed(component_name, suffixes):
    """
    Determines whether the directory should be re-built.
    """
    hash_file_path = os.path.join(config.HASH_STORE, component_name)
    saved_hash = b''
    try:
        with open(hash_file_path) as hash_file:
            saved_hash = hash_file.read()
    except:
        pass

    computed_hash = hash_dir(
            os.path.join(config.ROOT, component_name),
            suffixes)

    return saved_hash == computed_hash


def save_build_hash(component_name, suffixes):
    """
    Saves the hash of the directory as a component.
    """
    hash_file_path = os.path.join(config.HASH_STORE, component_name)
    with open(hash_file_path, 'w') as hash_file:
        hash_file.write(
                hash_dir(
                    os.path.join(config.ROOT, component_name),
                    suffixes))


def generate_setup_py(component_name, *, executable=False, src=True):
    """
    Generates a setup.py file for a component.
    """
    tpl = _template_env.get_template('setup.py')
    return tpl.render(component=component_name, executable=executable, src=src)


def _handle_pytest_exits(ctx, command_line):
    try:
        ctx.run(command_line)
    except UnexpectedExit as e:
        if e.result.exited != 5:  # XXX: comes from pytest github issues
            raise e
        # Swallow it to allow us to proceed, even if we don't have tests yet.


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
    ctx.run("flake8")


def pyinstall(ctx):
    """
    Installs a Python module into the current env.
    """
    print("Installing...")
    ctx.run("python setup.py install")


# TODO simplify by making dir == name
def build_python_package(ctx, name, executable=True):
    """
    Builds a Python module.
    """
    if files_changed(name, ['.py']):
        print("{} unchanged, skipping.".format(name))
        return

    ensure_product_env()
    with build_intermediate_dir(ctx, name) as intermediate:
        print("Generating Python package `{}`...".format(name))

        srcdir = os.path.join(config.ROOT, name, 'src')
        build_srcdir = os.path.join(intermediate, 'src')
        build_pkgdir = os.path.join(build_srcdir, name)
        ensure_dir(build_srcdir)
        shutil.copytree(srcdir, build_pkgdir)

        # Generate an __init__.py file
        ctx.run("touch {}".format(os.path.join(build_pkgdir, '__init__.py')))

        # Generate a setup.py file
        with open(os.path.join(intermediate, 'setup.py'), 'w') as setup_py:
            setup_py.write(
                    generate_setup_py(name, executable=executable))

        pylint(ctx)
        with product_env(ctx):
            pytest(ctx)
            pyinstall(ctx)

    save_build_hash(name, ['.py'])
    print("Done.")


@contextmanager
def product_env(ctx):
    """
    A context manager for activating the product venv.
    """
    ensure_product_env()
    with ctx.prefix("source {}/bin/activate".format(config.PRODUCT_ENV)):
        yield


@contextmanager
def workspace(ctx, *, subdir=''):
    """
    A context manager for the workspace root directory.
    """
    with ctx.cd('{}{}'.format(config.ROOT, subdir)):
        yield


@contextmanager
def build_intermediate_dir(ctx, name, cleanup=True):
    """
    A context manager that creates an intermediate build directory, then
    changes directory to it.

    """
    newdir = os.path.join(config.BUILD_INTERMEDIATE, name)
    ensure_dir(newdir)
    if os.listdir(newdir):
        shutil.rmtree(newdir)
        ensure_dir(newdir)

    with ctx.cd(newdir):
        yield newdir

    if cleanup:
        shutil.rmtree(newdir)
