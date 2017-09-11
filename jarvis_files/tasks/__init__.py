from invoke import task, Collection
import shutil
import os

import config
import buildlib
import third_party


@task
def dep(ctx):
    """
    "Pins" external pip dependencies and installs them into the product venv.
    """
    buildlib.ensure_product_env()
    third_party.ensure_lcm(ctx)
    # TODO add other third_party deps
    with buildlib.workspace(ctx):
        print("Pinning pip dependencies...")
        ctx.run("pip-compile --output-file external_requirements.txt external_requirements.in")  # noqa
        print("Installing pip dependencies...")
        with buildlib.product_env(ctx):
            ctx.run("pip install --upgrade pip", hide='out')
            ctx.run("pip install -r external_requirements.txt", hide='out')
            ctx.run("pip install -r {}/requirements.txt".format(
                config.JARVIS_ROOT), hide='out')

    print("Done.")


@task
def clean(ctx):
    """
    Wipes away the product venv and hash store,
    enabling a completely new build environment.
    """
    if os.path.isdir(config.BUILD_ROOT):
        shutil.rmtree(config.BUILD_ROOT)


from . import build, run  # noqa

ns = Collection()
ns.add_collection(build)
ns.add_collection(run)
ns.add_task(dep)
ns.add_task(clean)
