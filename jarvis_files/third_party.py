import os
import shutil

import buildlib
import config


def check_lcm(ctx):
    """
    Checks for the existence of LCM in the product venv.
    """
    # check for lcm-gen in both venvs
    if (not os.path.exists(buildlib.get_product_executable('lcm-gen')) or
            not os.path.exists(buildlib.get_jarvis_executable('lcm-gen'))):
        return False

    # check for liblcm.so
    # TODO Linux-only!
    if not os.path.exists(buildlib.get_product_file('lib', 'liblcm.so')):
        return False

    # check that lcm can be imported in Python inside product venv
    with buildlib.product_env(ctx):
        try:
            ctx.run("python -c 'import lcm'", hide='both')
        except:
            return False

    return True


def ensure_lcm(ctx):
    """
    Installs LCM into the product venv. LCM is expected to be included as a
    git submodule or a subdirectory in the third-party directory.

    Also installs lcm-gen into the Jarvis venv.
    """
    if check_lcm(ctx):
        print("LCM already installed, skipping.")
        return

    lcmdir = os.path.join(config.THIRD_PARTY_ROOT, 'lcm')
    with buildlib.build_intermediate_dir(ctx, 'lcm'):
        ctx.run("cp -r {}/* .".format(lcmdir))
        print("Configuring LCM...")
        ctx.run("./bootstrap.sh")
        ctx.run("./configure --prefix={}".format(config.PRODUCT_ENV))
        print("Building LCM...")
        ctx.run("make")
        print("Installing LCM...")
        ctx.run("make install")
        # Copy the lcm-gen binary into the Jarvis venv so it may be accessible
        # for other parts of the build process.
        shutil.copy("{}/bin/lcm-gen".format(config.PRODUCT_ENV),
                    "{}/bin/lcm-gen".format(config.JARVIS_ENV))

        # Install Python library
        with buildlib.product_env(ctx):
            with ctx.cd('lcm-python'):
                ctx.run("python setup.py install")

    print("Finished installing LCM.")
