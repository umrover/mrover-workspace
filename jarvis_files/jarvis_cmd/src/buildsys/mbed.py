import os

from . import BuildContext


BOARD_FILES = {
    'DISCO_L476VG': 'stm32l4discovery.cfg'
}


def generate_ocd_flash(ctx, dir_, name, board):
    """
    Generates an OpenOCD configuration for a component.
    """
    return ctx.wksp.template(
            'openocd.cfg',
            binfile_path=os.path.join(
                dir_, 'BUILD', board, 'GCC_ARM', name))


class MbedBuilder(BuildContext):
    def __init__(self, dir_, wksp, board, app=False, deps=[]):
        super().__init__(dir_, wksp, ['.cpp', '.h', '.hpp'])
        self.board = board
        self.app = app
        self.deps = deps

    def build(self):
        """
        Builds and installs an mbed OS project onto a microcontroller.
        """
        if not self.files_changed():
            print("{} unchanged, skipping.".format(self.dir_))
            return

        if not self.app:
            return

        with self.scratch_space() as intermediate:
            self.run("cp -r {}/* .".format(
                os.path.join(self.wksp.root, self.dir_)))
            print('Compiling mbed OS project...')
            # TODO clean this up
            mbed_project_dir = os.path.join(self.wksp.build_intermediate,
                                            'mbed-project')
            if not os.path.exists(os.path.join(intermediate, 'mbed-os')):
                self.run("ln -s {}/mbed-os .".format(mbed_project_dir))
            if not os.path.exists(os.path.join(intermediate,
                                               'mbed-os.lib')):
                self.run("ln -s {}/mbed-os.lib .".format(mbed_project_dir))
            if not os.path.exists(os.path.join(intermediate, '.mbed')):
                self.run("cp {}/.mbed .".format(mbed_project_dir))
            if not os.path.exists(os.path.join(intermediate,
                                               'mbed_settings.py')):
                self.run("ln -s {}/mbed_settings.py .".format(
                    mbed_project_dir))

            # Symlink dependent library code into the project dir
            for dep in self.deps:
                dep_dir = os.path.join(self.wksp.root, dep)
                with os.scandir(dep_dir) as it:
                    for entry in it:
                        if (entry.name.endswith('.h') or
                                entry.name.endswith('.hpp') or
                                entry.name.endswith('.c') or
                                entry.name.endswith('.cpp')):
                            os.symlink(entry.path,
                                       os.path.join(intermediate, entry.name))

            with self.wksp.inside_mbed_env():
                self.run('mbed target {}'.format(self.board))
                self.run('mbed compile')
            print('Compilation successful.')

            # Prepare OpenOCD
            with open(
                    os.path.join(intermediate, 'flash_code.cfg'),
                    'w') as ocd_flash:
                ocd_flash.write(
                        generate_ocd_flash(self, intermediate,
                                           os.path.basename(self.name),
                                           self.board))

            # Run OpenOCD
            with self.wksp.inside_mbed_env():
                try:
                    self.run(
                        'openocd -f board/{} '
                        '-f flash_code.cfg'.format(BOARD_FILES[self.board]))
                    print('Flashed to connected microcontroller.')
                except:
                    print('Flash failed.')

    def debug(self):
        # TODO run OpenOCD in the background
        # TODO and then launch a pre-configured gdb
        # TODO when gdb exits, kill OpenOCD
        raise RuntimeError('not implemented')
