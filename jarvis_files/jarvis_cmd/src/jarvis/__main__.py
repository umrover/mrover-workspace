import argparse
import sys
from buildsys import WorkspaceContext
from invoke.exceptions import UnexpectedExit

from .build import build_dir, clean, build_deps, build_all


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
    parser_build.add_argument('-o', '--option', dest='build_opt',
                              help='A build option to pass to the underlying '
                              'build system')
    parser_build.add_argument('-a', '--all', action='store_true', help='Build all')
    parser_build.add_argument('-n','--not-projects', nargs='+',
                              help='Don\'t build these projects when building all (-a).')

    subcommands.add_parser('clean',
                           help='Removes the product env')
    subcommands.add_parser('dep',
                           help='Installs 3rdparty folder into product env')
    subcommands.add_parser('exec',
                           help='Runs a command in the product venv')
    subcommands.add_parser('upgrade',
                           help='Re-installs the Jarvis CLI')

    args = parser.parse_args()

    try:
        ctx = WorkspaceContext(args.root_dir)

        if args.subcommand_name == 'build':
            build_deps(ctx)
            opt = None
            if args.build_opt:
                opt = args.build_opt.split('=')[0:2]
                print('option is {}'.format(opt))
            if args.all:
                return build_all(ctx, clean_dir_name(args.dir), opt, args.not_projects)
            else:
                build_dir(ctx, clean_dir_name(args.dir), opt)
        elif args.subcommand_name == 'clean':
            clean(ctx)
        elif args.subcommand_name == 'dep':
            build_deps(ctx)
    except UnexpectedExit as e:
        sys.exit(e.result.exited)


if __name__ == "__main__":
    main()
