import configparser
import os
import sys
import shutil
import hashlib
from buildsys import hash_file
from buildsys.python import PythonBuilder
from buildsys.lcm import LCMBuilder
from buildsys.rollupjs import RollupJSBuilder
from buildsys.meson import MesonBuilder
from buildsys.shell import ShellBuilder
from buildsys.config import ConfigBuilder

from . import third_party


def clean(ctx):
    """
    Deletes the product venv, requiring a full rebuild.
    """
    shutil.rmtree(ctx.product_env)
    shutil.rmtree(ctx.hash_store)


def get_builder(ctx, d, opt=None):
    project = os.path.join(ctx.root, d)
    project_cfg_path = os.path.join(project, 'project.ini')
    project_cfg = configparser.ConfigParser()

    project_cfg['build'] = {}
    project_cfg.read(project_cfg_path)

    build_defs = project_cfg['build']

    deps = [s.strip().rstrip()
            for s in build_defs.get('deps', '').split(',')]

    if len(deps) == 1 and deps[0] == '':
        deps.pop()

    for dep in deps:
        print('- building dependency {}'.format(dep))
        build_dir(ctx, dep)

    lang = build_defs.get('lang', '')

    if lang == 'python':
        print('Building Python 3 package')
        executable = build_defs.get('executable', 'False') == 'True'
        return PythonBuilder(d, ctx, executable)
    elif lang == 'js':
        print('Building JavaScript package')
        app = build_defs.get('app', 'False') == 'True'
        port = build_defs.get('port', None)
        return RollupJSBuilder(d, ctx, deps, app, port)
    elif lang == 'cpp':
        print('Building C++ package')
        return MesonBuilder(d, ctx, opt)
    elif lang == 'lcm':
        print('Building LCM package')
        return LCMBuilder(d, ctx)
    elif lang == 'shell':
        print('Building shell package')
        return ShellBuilder(d, ctx)
    elif lang == 'config':
        print('Building config package')
        return ConfigBuilder(d, ctx)
    else:
        print("error: unrecognized language '{}'".format(lang))
        sys.exit(1)


def build_dir(ctx, d, opt=None):
    """
    Builds the project in the given directory
    """
    if "./" == d[:2]:
        d = d[2:]
    get_builder(ctx, d, opt).build()
    print("Done")


def get_site_cfg():
    PACKAGE_NAMES = ['lcm', 'rapidjson']
    site_cfg_path = os.path.join(os.environ['HOME'], 'mrover.site')
    site_cfg = configparser.ConfigParser()
    site_cfg['third_party'] = {}
    site_cfg.read(site_cfg_path)
    tpdeps = site_cfg['third_party']
    return {pkg_name: tpdeps.get(pkg_name, '') != 'False'
            for pkg_name in PACKAGE_NAMES}


# TODO refactor this
def pip_deps_changed(ctx):
    hash_file_path = os.path.join(ctx.hash_store, 'external_requirements')
    saved_hash = b''

    try:
        with open(hash_file_path) as f:
            saved_hash = f.read()
    except:
        pass

    full_path = os.path.join(ctx.root, 'external_requirements.txt')
    computed_hash = hash_file(full_path, hashlib.sha256)

    return saved_hash != computed_hash


def save_pip_deps_hash(ctx):
    full_path = os.path.join(ctx.root, 'external_requirements.txt')
    hash_file_path = os.path.join(ctx.hash_store, 'external_requirements')
    with open(hash_file_path, 'w') as f:
        f.write(hash_file(full_path, hashlib.sha256))


def build_deps(ctx):
    """
    Build the dependencies. This is hard-coded for now.
    """
    site_cfg = get_site_cfg()
    ctx.ensure_product_env()
    if site_cfg['rapidjson']:
        third_party.ensure_rapidjson(ctx)
    if site_cfg['lcm']:
        third_party.ensure_lcm(ctx)
    if pip_deps_changed(ctx):
        with ctx.cd(ctx.root):
            print("Installing pip dependencies...")
            with ctx.inside_product_env():
                ctx.run("pip install --upgrade pip", hide='out')
                ctx.run("pip install -r external_requirements.txt", hide='out')
                ctx.run("pip install -r {}/requirements.txt".format(
                    ctx.jarvis_root), hide='out')
    else:
        print("pip dependencies already installed, skipping.")

    save_pip_deps_hash(ctx)
    print("Done.")


def build_all(ctx, d, opt, not_build):
    num_projects = 0
    failed_projects = []

    if not_build is None:
        not_build = []

    for root, dirs, files in os.walk(d):
        dirs[:] = list(filter(lambda x: ".mrover" != x, dirs))
        if "project.ini" in files and ".mrover" not in root \
            and len([i for i in not_build if i in root]) == 0:
            num_projects += 1
            print("Building: ", root)
            try:
                build_dir(ctx, root, opt)
            except Exception as e:
                failed_projects.append((root, e))
    if len(not_build):
        print("Did not build project(s):")
        for i in not_build:
            print(i)
    if len(failed_projects):
        print("Failed on project(s):")
        for prj in failed_projects:
            print(prj[0])
            print("Exception: {}",format(prj[1]))
    print("Successfully built: {} of {} project(s).".format(
        num_projects - len(failed_projects), num_projects))
    return len(failed_projects)
