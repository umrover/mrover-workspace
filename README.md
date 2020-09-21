# MRover

This is the repository for all custom software written for the University of
Michigan Mars Rover team (MRover). The repository is largely self-contained,
with the MRover 2017-18 build system `jarvis` contained in the source tree.

## Development

It is recommended that you use our [Vagrant](https://vagrantup.com) box. This
will set up a virtual machine for development using the Ansible configurations
that we have to set up the production base station and production onboard
computer. To run it, clone this repository, then run the following commands in
the directory provided.

```sh
$ vagrant up
$ vagrant ssh
```

If you would prefer not to use a virtual machine for development, you may use
the [Ansible configurations](./ansible/README.md) provided to configure a
Ubuntu 16.04-based system. **No other development environments will be
supported.**

## Running the Software (Ubuntu only)

Begin by cloning this `git` repository:

```sh
$ git clone https://github.com/umrover/mrover-workspace.git
$ cd mrover-workspace
```

Next, force Jarvis to bootstrap itself:

```sh
$ ./jarvis
```

The output of the previous command will list all the buildable components in
this repository. They have a one-to-one correspondence with the directories at
the root of this repository.

Choose some components to build and build them:

```sh
$ ./jarvis build jetson/teleop
$ ./jarvis build base_station/gui
...
```

Once you have built the components, you can run them easily with `jarvis exec`.
Note that the slashes are converted to underscores in the executable name.

```sh
$ ./jarvis exec jetson/teleop
$ ./jarvis exec base_station/gui
...
```

In the future, it will be easy to compile an entire subtree, such as `jetson`.

## Jarvis In-depth Documentation

See [here](jarvis_files/README.md).

## Contribution Workflow

**Assumption**: You have configured your [SSH keys with GitHub](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/).
Additionally, you have a working knowledge of the Unix command line and have
set up Git for your operating system.

Begin by forking this repository. This will give you your own workspace in
which to make contributions.


Next, clone your fork:

```sh
$ git clone git@github.com:<your-github-username>/mrover-workspace.git
$ cd mrover-workspace
```

Add the official MRover repository as upstream:

```sh
$ git remote add upstream https://github.com/umrover/mrover-workspace.git
```

Now, you are ready to develop.

Before you make your changes, ensure that they still work by building the
component you modified. Jarvis automatically tests and style-checks your code
as part of a normal build, so if the system builds with your changes, you are
ready to submit them.

Once you've made a change that you're ready to contribute, commit it and push
it to your fork (called `origin` in git).  Afterwards, open a pull request in
the GitHub UI. This will be merged by an MRover Software Lead after a short
review.
