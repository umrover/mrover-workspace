from invoke import task
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


@task(dep)
def rover_msgs(ctx):
    """
    Builds the LCM datatypes in the `rover_msgs` directory.
    """
    if buildlib.files_changed('rover_msgs', ['.lcm']):
        print("rover_msgs unchanged, skipping.")
        return

    # Get all LCM files
    lcm_files = [
            f.path
            for f in os.scandir(config.LCM_TYPES_ROOT)
            if os.path.splitext(f.name)[1] == '.lcm'
    ]
    lcm_files_cmdline = ' '.join('"{}"'.format(v) for v in lcm_files)

    buildlib.ensure_product_env()
    # Generate the Python stuff
    with buildlib.build_intermediate_dir(ctx, 'rover_msgs_py') as intermediate:
        print("Generating Python package `rover_msgs`...")
        ctx.run("lcm-gen --python {} --ppath {}".format(
            lcm_files_cmdline, intermediate))

        # Generate a setup.py file
        with open(os.path.join(intermediate, 'setup.py'), 'w') as setup_py:
            setup_py.write(buildlib.generate_setup_py('rover_msgs', src=False))

        # Install the package
        with buildlib.product_env(ctx):
            buildlib.pyinstall(ctx)

    print("Finished generating Python package.")

    # Generate C++ stuff
    with buildlib.build_intermediate_dir(
            ctx, 'rover_msgs_cpp') as intermediate:
        print("Generating C++ package `rover_msgs`...")
        ctx.run("lcm-gen --cpp {} --cpp-std c++14".format(lcm_files_cmdline))

        # Copy *.hpp to the include dir
        target_dir = os.path.join(config.PRODUCT_ENV, 'include', 'rover_msgs')
        if os.path.isdir(target_dir):
            shutil.rmtree(target_dir)
        shutil.copytree(os.path.join(intermediate, 'rover_msgs'), target_dir)

    print("Finished generating C++ package.")

    buildlib.save_build_hash('rover_msgs', ['.lcm'])
    print("Done.")


@task(rover_msgs)
def rover_common(ctx):
    """
    Builds the `rover_common` library.
    """
    buildlib.build_python_package(ctx, 'rover_common', executable=False)


@task(rover_common)
def onboard_teleop(ctx):
    """
    Builds the `onboard_teleop` executable.
    """
    buildlib.build_python_package(ctx, 'onboard_teleop')


@task(rover_msgs)
def nav(ctx):
    """
    Builds the `nav` executable.
    """
    if buildlib.files_changed('nav', ['.h', '.hpp', '.c', '.cpp']):
        print("nav unchanged, skipping")
        return

    buildlib.ensure_product_env()
    with buildlib.build_intermediate_dir(ctx, 'nav') as intermediate:
        print("Generating C++ package `nav`...")

        # Get all CPP files
        cpp_files = [
                f.path
                for f in os.scandir(os.path.join(config.ROOT, 'nav'))
                if os.path.splitext(f.name)[1] in [
                    '.c', '.cpp'
                ]
        ]
        cpp_files_cmdline = ' '.join('"{}"'.format(v) for v in cpp_files)

        pkg_cfg_path = ctx.run(
                "pkg-config --variable pc_path pkg-config").stdout.strip()
        pkg_cfg_path = '{}:{}'.format(
                os.path.join(config.PRODUCT_ENV, 'lib', 'pkgconfig'),
                pkg_cfg_path)

        with buildlib.workspace(ctx, subdir='nav'):
            ctx.run("PKG_CONFIG_PATH={} meson --prefix {} {}".format(
                pkg_cfg_path, config.PRODUCT_ENV, intermediate))

            # Run clang-tidy
            print("Linting...")
            ctx.run("clang-tidy {} -- -I{}".format(
                cpp_files_cmdline,
                os.path.join(config.PRODUCT_ENV, 'include')))

        # Run test cases
        print("Testing...")
        ctx.run("ninja test")

        # Install
        print("Installing...")
        ctx.run("ninja install")

    buildlib.save_build_hash('nav', ['.h', '.hpp', '.c', '.cpp'])
    print("Done!")


@task(rover_common)
def base_station_bridge(ctx):
    """
    Builds the `base_station_bridge` executable.
    """
    buildlib.build_python_package(ctx, 'base_station_bridge',
                                  directory=os.path.join(
                                      'base_station', 'bridge'))


@task(base_station_bridge)
def base_station_gui(ctx):
    """
    Builds the `base_station_gui` executable.
    """
    bsgui_root = os.path.join('base_station', 'gui')
    buildlib.ensure_product_env()
    with buildlib.build_intermediate_dir(
            ctx, 'base_station_gui') as intermediate:
        print("Building `base_station_gui` with yarn...")

        srcdir = os.path.join(config.ROOT, bsgui_root)
        shutil.copytree(srcdir,
                        os.path.join(intermediate, 'src_cpy'),
                        ignore=shutil.ignore_patterns('node_modules'))

        with ctx.cd("src_cpy"):
            ctx.run("yarn")
            ctx.run("yarn run build")

        bsgui_dir = os.path.join(config.PRODUCT_ENV, 'share', 'bsgui')
        if os.path.isdir(bsgui_dir):
            shutil.rmtree(bsgui_dir)
        shutil.copytree(os.path.join(intermediate, 'src_cpy', 'dist'),
                        os.path.join(bsgui_dir, 'dist'))
        shutil.copyfile(os.path.join(intermediate, 'src_cpy', 'index.html'),
                        os.path.join(bsgui_dir, 'index.html'))

        # Copy binary
        bsgui_exec_path = os.path.join(config.PRODUCT_ENV,
                                       'bin', 'base_station_gui')
        with open(bsgui_exec_path, 'w') as bsgui_exec:
            bsgui_exec.write(buildlib.template('base_station_gui',
                                               bsgui_dir=bsgui_dir))
        os.chmod(bsgui_exec_path, 0o755)

    print("Done.")


@task(base_station_bridge, base_station_gui)
def base_station(ctx):
    """
    Builds all base station executables.
    """
    pass


# TODO add hwi_hughey
@task(onboard_teleop, nav)
def onboard_hughey(ctx):
    """
    Builds Hughey's onboard software stack.
    """
    pass


# TODO add hwi_18
@task(onboard_teleop, nav)
def onboard_18(ctx):
    """
    Builds Eighteen's onboard software stack.
    """
    pass
