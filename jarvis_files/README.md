# Jarvis Build System

Jarvis is a build system optimized for MRover. It is designed to automate many
tasks in assembling pieces of the rover software stack. It is implemented in
Python 3 and uses a simple shell script to bootstrap itself.

## Quickstart

From the root directory of this repository, run `./jarvis`, which will create
the Jarvis `venv`, populate it, and display the list of Jarvis tasks.

**NOTE:** It is recommended to add an alias to your `~/.bashrc` or equivalent
to assist in invoking Jarvis. This has the benefit of being able to invoke
Jarvis from anywhere in the filesystem. This step is already done by our
Vagrant box.

Example for `bash`:

```sh
# ~/.bashrc
alias jarvis='$HOME/mrover-workspace/jarvis'
```

## Design

Jarvis creates two virtual environments, one for Jarvis's own dependencies
(called the "Jarvis `venv`"), and one for products of the build, called the
"product `venv`". The product `venv` is used as the target destination for
all parts of build -- C/C++ output executables are installed into the product
`venv` as well as Python executables.

## Usage

- `jarvis dep` :: Compile and install 3rd-party C/C++ dependencies (located 
  in `3rdparty/`) using instructions specified in the `third_party.py` file.
  After installing C/C++ dependencies, Jarvis will run `pip-compile` on the
  contents of `external_requirements.in` to produce
  `external_requirements.txt`, which will then be installed into the product
  `venv`.
- `jarvis exec <command>` :: Runs a command in the product `venv`.
- `jarvis clean` :: Wipes away the product `venv`, requiring a full re-build.
- `jarvis implode` :: Wipes away the Jarvis `venv`, requiring a full re-bootstrap.
- `jarvis build rover_msgs` :: Compiles the LCM descriptions in `rover_msgs/`
  into both C++ and Python libraries and installs them into the product `venv`.
- `jarvis build <target>` :: Lint, compile, test, then install `<target>` into
  the product `venv` using the procedure described below.

### Targets

All other Jarvis projects (anything with a `project.ini` file) execute a lint,
then a compile, then a test, then an install into the product `venv`. 

In Python projects, the lint is carried out by `flake8`, in C++, `clang-tidy` 
is used.

Compiling Python is a no-op; compiling C++ will invoke the `meson` build
system. 

In Python, the testing system will run `doctest` on all files located in the
`src/` directory, and it will run `pytest` on the test cases located in the
`test/` directory. In C++, `meson` has its own mechanism for registering test
cases, and that is used.

The products are then *installed* into the product `venv`. In Python, an
executable is generated from the function `main` located in `src/__main__.py`,
and in C++, executables generated from `meson` are installed into the
appropriate locations.

## Technical Documentation

Technical documentation for the Jarvis build system is a work in progress. The
Python docstrings provided for the functions in `buildsys/` ought to be
moderately useful for anyone who wishes to work on Jarvis, but improvements are
in the works.

## Future goals

Quality of life improvements, and a smaller, more orthogonal codebase.
