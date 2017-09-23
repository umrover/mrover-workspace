import configparser
import os
import sys
import shutil
from buildsys.python import PythonBuilder
from buildsys.lcm import LCMBuilder
from buildsys.mbed import MbedBuilder

from . import third_party


def clean(ctx):
    """
    Deletes the product venv, requiring a full rebuild.
    """
    shutil.rmtree(ctx.build_root)


def build_dir(ctx, d):
    """
    Builds the project in the given directory
    """
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
        executable = bool(build_defs.get('executable', 'True'))
        builder = PythonBuilder(d, ctx, executable)
    elif lang == 'node':
        print('Building Node.js package NOT IMPLEMENTED')
    elif lang == 'cpp':
        print('Building C++ package NOT IMPLEMENTED')
    elif lang == 'lcm':
        print('Building LCM package')
        builder = LCMBuilder(d, ctx)
    elif lang == 'mbed':
        print('Building mbed OS package')
        builder = MbedBuilder(d, ctx)
    else:
        print("error: unrecognized language '{}'".format(lang))
        sys.exit(1)

    builder.build()
    print("Done")


def build_deps(ctx):
    """
    Build the dependencies. This is hard-coded for now.
    """
    # TODO make this not hard-coded
    ctx.ensure_product_env()
    third_party.ensure_lcm(ctx)
    third_party.ensure_mbed_cli(ctx)
    # TODO add other third-party deps
    with ctx.cd(ctx.root):
        print("Pinning pip dependencies...")
        ctx.run("pip-compile --output-file external_requirements.txt external_requirements.in")  # noqa
        print("Installing pip dependencies...")
        with ctx.inside_product_env():
            ctx.run("pip install --upgrade pip", hide='out')
            ctx.run("pip install -r external_requirements.txt", hide='out')
            ctx.run("pip install -r {}/requirements.txt".format(
                ctx.jarvis_root), hide='out')

    print("Done.")
