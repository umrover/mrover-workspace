import os

from . import BuildContext


def generate_ocd_flash(ctx, dir_, name):
    """
    Generates an OpenOCD configuration for a component.
    """
    return ctx.wksp.template(
            'openocd.cfg',
            binfile_path=os.path.join(
                dir_, 'BUILD', 'DISCO_L476VG', 'GCC_ARM', name))


class MbedBuilder(BuildContext):
    def __init__(self, dir_, wksp):
        super().__init__(dir_, wksp, ['.cpp', '.h', '.hpp'])

    def build(self):
        """
        Builds and installs an mbed OS project onto a microcontroller.
        """
        if not self.files_changed():
            print("{} unchanged, skipping.".format(self.dir_))
            return

        # TODO make the board configurable
        with self.scratch_space() as intermediate:
            self.run("cp -r {}/* .".format(
                os.path.join(self.wksp.root, self.dir_)))
            print('Compiling mbed OS project...')
            with self.wksp.inside_mbed_env():
                self.run('mbed new .')
                self.run('mbed target DISCO_L476VG')
                self.run('mbed toolchain GCC_ARM')
                self.run('mbed compile')
            print('Compilation successful.')

            # Prepare OpenOCD
            with open(
                    os.path.join(intermediate, 'flash_code.cfg'),
                    'w') as ocd_flash:
                ocd_flash.write(
                        generate_ocd_flash(self, intermediate,
                                           os.path.basename(self.name)))

            # Run OpenOCD
            with self.wksp.inside_mbed_env():
                try:
                    self.run(
                        'openocd -f board/stm32l4discovery.cfg '
                        '-f flash_code.cfg')
                    print('Flashed to connected microcontroller.')
                except:
                    print('Flash failed.')
