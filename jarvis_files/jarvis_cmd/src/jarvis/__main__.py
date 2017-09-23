import argparse
import sys
from buildsys import WorkspaceContext
from invoke.exceptions import UnexpectedExit

from .build import build_dir, clean, build_deps


def main():
    parser = argparse.ArgumentParser(
            description='Jarvis build system for MRover')
    parser.add_argument('-r', '--root', dest='root_dir',
                        help='give the root directory of the workspace')

    subcommands = parser.add_subparsers(
            title='Subcommands',
            description='valid subcommands',
            help='Actions',
            dest='subcommand_name')
    parser_build = subcommands.add_parser('build', help='Build a directory')
    parser_build.add_argument('dir', help='The directory to build')

    parser_run = subcommands.add_parser('run',
                                        help="Run one machine's configuration")
    parser_run.add_argument('machine')

    subcommands.add_parser('clean',
                           help='Removes the product env')
    subcommands.add_parser('dep',
                           help='Installs 3rdparty folder into product env')
    subcommands.add_parser('exec',
                           help='Runs a command in the product venv')

    parser_mbed = subcommands.add_parser('mbed',
                                         help='Runs the mbed CLI')
    parser_mbed.add_argument('mbed_args', nargs='+')

    args = parser.parse_args()

    try:
        ctx = WorkspaceContext(args.root_dir)

        if args.subcommand_name == 'build':
            # TODO add ensure_deps
            build_dir(ctx, args.dir)
        elif args.subcommand_name == 'run':
            # TODO
            print('NOT IMPLEMENTED YET')
        elif args.subcommand_name == 'clean':
            clean(ctx)
        elif args.subcommand_name == 'dep':
            build_deps(ctx)
        elif args.subcommand_name == 'mbed':
            with ctx.inside_mbed_env():
                ctx.run('mbed {}'.format(
                    ' '.join('"{}"'.format(arg) for arg in args.mbed_args)))
    except UnexpectedExit as e:
        sys.exit(e.result.exited)


if __name__ == "__main__":
    main()
