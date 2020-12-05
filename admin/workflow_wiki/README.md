# MRover Workflow Wiki

## Table of Contents
- [MRover Workflow Wiki](#mrover-workflow-wiki)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Making Code Changes](#making-code-changes)
    - [Assumptions](#assumptions)
    - [Details](#details)
    - [Example](#example)
  - [Merging Code Changes into Main](#merging-code-changes-into-main)
    - [Assumptions](#assumptions-1)
    - [Details](#details-1)
    - [Example](#example-1)

## Overview
This document describes the process by which we recommend contributors should use when making code changes for our repo and how to go about getting those changes merged into our `main` branch.
This is not the only way to make or merge changes but, since many of our members are still learning Git, this process is standardized across the team and will allow you to spend less time dealing with nasty Git problems such as merge conflicts and how to avoid
unnecessarily huge review processes.
Additionally, using this workflow will help leads to help you use Git.
This document requires a working knowledge of Git. Please feel free to stop and look-up Git methods you don't fully understand as you come across them.

## Making Code Changes
This section describes the team's workflow from the point of getting assigned
an issue to just before you make a pull request.

### Assumptions
We make the following assmptions in this tutorial.
If any of there are not true, please resolve this before proceeding.
1. You have set up your MRover enviromnent. This can be done by following along with our [Dev Environment Setup tutorial](https://docs.google.com/document/d/1l5zq6cyX115KAdGTMcv_rSFH0cFxCucFgVcFn7b-KnI/edit#heading=h.j1toowrjx1y4).
2. You have read and completed all the steps laid out in the [main README](/README.md).
3. Your `upstream` remote points to `https://github.com/umrover/mrover-workspace.git` or `git@github.com:umrover/mrover-workspace.git`. This can be checked with `git remote -v`.
4. Your `origin` remote points to `https://github.com/<your-github-username>/mrover-workspace.git` or `git@github.com:<your-github-username>/mrover-workspace.git`. This can be checked with `git remote -v`.

### Details


### Example
For this demo, we will be resolving
[issue 431](https://github.com/umrover/mrover-workspace/issues/431). This
example workflow is assuming that you have done your previous workflow using
this method otherwise there might be a few extra steps (in which case you should
reach out to a lead or SAM for more help if needed). If you’re just starting
out, you should be fine to follow along.

This example is intended to demonstrate the process rather than being something
we expect you to recreate the changes. Our suggestion is to use this as a guide
when making changes of your own and for you to refer back to this throughout.
We expect that this guide will be more useful to you as you are making changes
and likely will be too dense to remember everything when you first read it.

1. Create a new branch to work off of.
    * First, I want to base my changes off of the most up to date
      `upstream/main` branch. To do so, I check out my `main` branch and then
      I pull from `upstream/main`. _Note that `upstream` is my local name for_
      _the umrover remote (set up in the_
      _[main README](/README.md#contribution-workflow))._
      <br/>
      ![git checkout main, git pull upstream main](/admin/workflow-wiki/img/git_update_main.png)
    * Next, I want to create a new branch to do my code changes on that is
      based off of `main`/`upstream/main` (they are the same right now).
      I will call my branch `hotkey-docs`.
      <br/>
      ![git checkout -b hotkey-docs](/admin/workflow-wiki/img/git_new_branch.png)

2. Make your code changes.
    * First work session
        * If not already on my new branch (`hotkey-docs`), I check it out.
          Note that this did nothing (and told me so) since I was already on
          `hotkey-docs`.
          <br/>
          ![work session 1, git checkout hotkey-docs](/admin/workflow-wiki/img/git_checkout_branch.png)
        * Next, I make sure I am working off of the most up to date code.
          I do this with `git pull --rebase upstream main`. Note that most likely this will have no effect since I just created this branch off the most up to date version of `upstream/main`.
          <br/>
          ![work session 1, git pull --rebase upstream main](/admin/workflow-wiki/img/ws1_git_pull_rebase.png)
        * *Make the actual changes to code.*
        * *Test the code changes.*
        * Checking `git status` often is a great habit to have. This can be done
          by typing `git status`.
          <br/>
          ![work session 1, git status](/admin/workflow-wiki/img/ws1_git_status_1.png)
        * Now I want to stage my changes for commit. I do this with `git add`.
          I notice from `git status` that all my changes are in
          `simulators/nav/src/components/hotkeys.md` so I can add that file by
          typing `git add simulators/nav/src/components/hotkeys.md`.
          Note that I could have used `git add .` but I did not because it’s a
          good habit to think about what we want to be staging rather than
          blindly staging all changes.
          Using `git diff` is also a good tool here to see what was changed
          about those files.
          This might allow me to see if I forgot to remove a debug print
          statement, for example.
          <br/>
          ![work session 1, git add simulators/nav/src/components/hotkeys.md](/admin/workflow-wiki/img/ws1_git_add.png)
        * Next I commit my changes using `git commit`. I type `git commit` at
          the command line and then type my commit message in the text editor
          that automatically opens up. Note that I do not intend for this to be
          my final commit message but I do want it to be useful so I can
          determine what I did in this commit.
          <br/>
          ![work session 1, git commit message](/admin/workflow-wiki/img/ws1_git_commit_message.png)
          <br/>
          ![work session 1, git commit, git status](/admin/workflow-wiki/img/ws1_git_commit.png)
        * I can type `git log` to view the change in the commit history. This is
          one I like to do pretty often just like `git status`.
          <br/>
          ![work session 1, git log](/admin/workflow-wiki/img/ws1_git_log.png)
        * Lastly, I push my changes, so that I don’t lose any work if something
          were to happen to my local repository, using
          `git push origin hotkey-docs`.
          <br/>
          ![work session 1, git push origin hotkey-docs](/admin/workflow-wiki/img/ws1_git_push.png)

    * Second work session
        * If not already on my new branch (`hotkey-docs`), I check it out.
        * Next, I make sure I am working off of the most up to date version of
          `upstream/main`.
          I do this with `git pull --rebase upstream main`. Note that most likely
          this will have no effect since I just created this branch off the most
          up to date version of `upstream/main`.
          However, let's say I made a few code changes before `pull`ing.
          In this case, I could `stash` them, then `pull --rebase`, and then, if
          this changes are for
          [issue 431](https://github.com/umrover/mrover-workspace/issues/431), I
          un`stash` (`stash pop`) them.
          As the error message states, I could also commit them if I wanted.
          <br/>
          ![work session 2, git pull --rebase upstream main](/admin/workflow-wiki/img/ws2_git_pull_rebase.png)
        * *Make the actual changes to code.*
        * *Test the code changes.*
        * Again, I stage, commit, and push my changes. I see that all my changes
          are in `simulators/nav/src/components/ and there are no changes in
          that directory that I don't want so I can `add` that entire directory.
          <br/>
          ![work session 2, git add, git commit, git push origin hotkey-docs](/admin/workflow-wiki/img/ws2_git_add_commit_push.png)
          <br/>
          ![work session 2, git commit message](/admin/workflow-wiki/img/ws2_git_commit_message.png)
        * Using `git log`, I now see my two commits based off of
          `upstream/main`.
          <br/>
          ![work session 2, git log](/admin/workflow-wiki/img/ws2_git_log.png)

    * Third work session
        * Hopefully by now this is getting repetitive to you!
          This is what we want - a process that is easy to remember and do.
        * If not already on my new branch (`hotkey-docs`), I check it out.
        * Next, I make sure I am working off of the most up to date version of
          `upstream/main` using  `git pull --rebase upstream main`.
        * *Make the actual changes to code.*
        * *Test the code changes.*
        * Again, I stage, commit, and push my changes.
          <br/>
          ![work session 3, git add, git commit, git push origin hotkey-docs](/admin/workflow-wiki/img/ws3_git_add_commit_push.png)
          <br/>
          ![work session 3, git commit message](/admin/workflow-wiki/img/ws3_git_commit_message.png)
          <br/>
          ![work session 3, git log](/admin/workflow-wiki/img/ws3_git_log.png)

    * Note that a work session does not need to be on different days or even at
      different times. It is a good habit to be committing often.
      In fact, I actually did all my "work sessions" on the same day without
      stopping between them.

3. Testing
    * I system and integration test my changes. If any bugs came up, I would
      repeat step 2 and add more commits onto `origin hotkey-docs`.
    * For my changes, this involves making sure the documentation renders
      correctly in various places, but for most changes this would involve more
      intensive testing such as onboard rover testing, simulator testing, etc.
      For details on how to test your specific code, see one of the team leads
      or SAMs.

## Merging Code Changes into Main

### Assumptions

### Details

### Example
