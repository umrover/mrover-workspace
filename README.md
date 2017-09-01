# MRover

This is the repository for all custom software written for the University of
Michigan Mars Rover team (MRover). The repository is largely self-contained,
with the MRover 2017-18 build system `jarvis` contained in the source tree.

## Install System Requirements

### macOS
```sh
brew install python3 glib pkg-config ninja llvm yarn
echo 'PATH="/usr/local/opt/llvm/bin:${PATH}"' >> ~/.bash_profile
source ~/.bash_profile
```

### Ubuntu
```sh
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt update
sudo apt install python3-dev build-essential libglib2.0-dev ninja-build clang-tidy nodejs yarn
```

### Fedora
```sh
sudo wget https://dl.yarnpkg.com/rpm/yarn.repo -O /etc/yum.repos.d/yarn.repo
curl --silent --location https://rpm.nodesource.com/setup_8.x | sudo bash -
sudo dnf install python3-devel glib2-devel gcc gcc-c++ ninja-build clang clang-tools-extra nodejs yarn
```

## Running the Software

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
$ ./jarvis onboard_teleop
$ ./jarvis base_station
...
```

Once you have built the components, you can run them easily with `jarvis exec`:

```sh
$ ./jarvis exec onboard_teleop
$ ./jarvis exec base_station
...
```

## Jarvis In-depth Documentation

See [here](jarvis_files/README.md).

## Contribution Workflow

***Currently requires Linux. A VM is acceptable. WSL is YMMV.***

**Assumption**: You have configured your [SSH keys with GitHub](https://help.github.com/articles/adding-a-new-ssh-key-to-your-github-account/).
Additionally, you have a working knowledge of the Unix command line.

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
