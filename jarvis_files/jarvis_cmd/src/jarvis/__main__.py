import argparse
import sys
from buildsys import WorkspaceContext
from invoke.exceptions import UnexpectedExit

from .build import build_dir, clean, build_deps, debug_dir


def clean_dir_name(d):
    if d[-1] == '/':
        return d[:-1]
    return d


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

    # TODO add C++ projects
    parser_debug = subcommands.add_parser(
            'debug',
            help="Launch an mbed project in debug mode")
    parser_debug.add_argument('dir', help='The directory to debug')

    subcommands.add_parser('clean',
                           help='Removes the product env')
    subcommands.add_parser('dep',
                           help='Installs 3rdparty folder into product env')
    subcommands.add_parser('exec',
                           help='Runs a command in the product venv')
    subcommands.add_parser('upgrade',
                           help='Re-installs the Jarvis CLI')

    parser_mbed = subcommands.add_parser('mbed',
                                         help='Runs the mbed CLI')
    parser_mbed.add_argument('mbed_args', nargs='+')

    args = parser.parse_args()

    try:
        ctx = WorkspaceContext(args.root_dir)

        if args.subcommand_name == 'build':
            build_deps(ctx)
            build_dir(ctx, clean_dir_name(args.dir))
        elif args.subcommand_name == 'run':
            # TODO
            print('NOT IMPLEMENTED YET')
        elif args.subcommand_name == 'clean':
            clean(ctx)
        elif args.subcommand_name == 'debug':
            build_deps(ctx)
            build_dir(ctx, clean_dir_name(args.dir))
            debug_dir(ctx, clean_dir_name(args.dir))
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
