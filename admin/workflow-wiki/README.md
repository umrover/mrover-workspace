# MRover Workflow Wiki

## Table of Contents
[Overview](#overview)<br/>
[Making Code Changes](#making-code-changes)<br/>
[Making Code Changes - Example](#making-code-changes---example)<br/>
[Merging Code Changes into Main](#merging-code-changes-into-main)<br/>
[Merging Code Changes into Main Example](#merging-code-changes-into-main---example)<br/>

## Overview
This document describes how the process by which we recommend contributors
should use when making code changes for our repo and how to go about getting
those changes merged into our `main` branch. This is not the only way to make
or merge changes but, since many of our members are still learning Git, this
process is standardized across the team and will allow you to spend less time
dealing with nasty Git problems such as merge conflicts and how to avoid
unnecessarily huge review processes. This document requires a working knowledge
of Git. Please feel free to stop and look-up Git methods you don't fully
understand as you come across them.

## Making Code Changes
This section describes the team's workflow from the point of getting assigned
an issue to just before you make a pull request.

### Making Code Changes - Assumptions

### Making Code Changes - Details

### Making Code Changes - Example
For this demo, we will be resolving
[issue 395](https://github.com/umrover/mrover-workspace/issues/395) in the
[mrover-workspace repo](https://github.com/umrover/mrover-workspace). This
example workflow is assuming that you have done your previous workflow using
this method otherwise there might be a few extra steps (in which case you should
reach out to a lead or SAM for more help if needed). If you’re just starting
out, you should be fine to follow along.

This example is intended to demonstrate the process rather than being something
we expect you to recreate the changes. Our suggestion is to use this as a guide
when making changes of your own and for you to refer back to this throughout.
We expect that this guide will be more useful to you as you are making changes
and likely will be too dense to remember everything when you first read it.

**NOTE: The example images are repurposed from an older workflow guideline.**
**Where they say `nav-main`, replace it with `main`.**

1. Create a new branch to work off of. We show two methods here to hopefully
   give you a better understanding of what is actually going on but both methods
   are equivalent.
    * Method 1
        - First, I want to be working off of the `main` branch. This assumes
          that your `main` is set up to track `upstream/main` which it will be
          if you do not manually create a `main` branch locally. If you have
          created one locally, just set it to track upstream/main
          (`git branch -u upstream/main`).
          <br/>
          ![git checkout main](/admin/workflow-wiki/img/git_checkout_main.png)
        - I want to be working off the most up to date code so, as suggested by
          git, I will do a `git pull`. Again, note this assumes `upstream/main`
          is the tracking branch.
          <br/>
          ![git pull 1](/admin/workflow-wiki/img/git_pull_1.png)
        - Next, I want to create a new branch to do my code changes on that is
          based off of `main`. I will call my branch `start-pt`.
          <br/>
          ![git checkout -b start_point](/admin/workflow-wiki/img/git_checkout_start_point_1.png)
    * Method 2<br/>
        - First I will get the latest version of the repository. Note that this
          simply lets your local repository know about the changes to the remote repository but does not make changes to the local repository. Also
          note that there is no output because I already performed a `git fetch`
          (under the hood) when I did a `git pull` in method 1.
          <br/>
          ![git fetch upstream](/admin/workflow-wiki/img/git_fetch_upstream_1.png)
        - Now, I want to create a new branch to work off of that matches
          `upstream/main`.
          <br/>
          ![git checkout -b start_point](/admin/workflow-wiki/img/git_checkout_start_point_2.png)
        - Lastly, I don’t want to push changes from `start-pt` to `origin/main`
          (I will eventually push to `origin/start-pt`) so I unset upstream.
          <br/>
          ![git branch unset-upstream](/admin/workflow-wiki/img/git_branch_unset_upstream.png)

2. Make your code changes.
    * First work session
        - *Make the actual changes to code.*
        - *Test the code changes.*
        - Checking `git status` often is a great habit to have. This can be done
          by typing `git status`. Note that I have aliases `status` to `st` so
          that I can do this frequent command more quickly so you may see that
          used throughout.
          <br/>
          ![git status 1](/admin/workflow-wiki/img/git_status_1.png)
        - Now I want to stage my changes for commit. I do this with `git add`.
          I notice from `git status` that all my changes are in
          `simulators/nav/src/` and there is nothing in that directory that I
          don’t want to stage so I can add that entire directory by typing
          `git add simulators/nav/src/`. Note that I could have used `git add .`
          but I did not because it’s a good habit to think about what we want to
          be staging rather than blindly staging all changes. Using `git diff`
          is also a good tool here to see what was changed about those files.
          This might allow me to see if I forgot to remove a debug print
          statement, for example.
          <br/>
          ![git add simulators/nav/src/](/admin/workflow-wiki/img/git_add_1.png)
          ![git status 2](/admin/workflow-wiki/img/git_status_2.png)
        - Next I commit my changes using `git commit`. I type `git commit` at
          the command line and then type my commit message in the text editor
          that automatically opens up. Note that I do not intend for this to be
          my final commit message but I do want it to be useful so I can
          determine what I did in this commit.
          <br/>
          ![git commit message work session 1](/admin/workflow-wiki/img/git_commit_message_1.png)
          <br/>
          ![git status 3](/admin/workflow-wiki/img/git_status_3.png)
        - I can type `git log` to view the change in the commit history. This is
          one I like to do pretty often just like `git status`.
          <br/>
          ![git log 1](/admin/workflow-wiki/img/git_log_1.png)
        - Lastly, I push my changes, so that I don’t lose any work if something
          were to happen to my local repository, using
          `git push origin start-pt`.
          <br/>
          ![git push 1](/admin/workflow-wiki/img/git_push_1.png)

    * Second work session
        - I start by making sure I am working off of the most up to date code.
          I do this with `git pull --rebase upstream main`.
          <br/>
          ![git pull rebase 1](/admin/workflow-wiki/img/git_pull_rebase_1.png)
          If I had made any code changes since my last commit, I could
          `git stash` those changes, then `git pull --rebase upstream main`,
          then `git stash pop` them.
          <br/>
          ![git pull stash, pull, stash pop](/admin/workflow-wiki/img/git_stash_pull_unstash_1.png)
        - *Make the actual changes to code.*
        - *Test the code changes.*
        - Again, I stage, commit, and push my changes.
          <br/>
          ![git add commit push 1](/admin/workflow-wiki/img/git_add_cm_push_1.png)
          <br/>
          ![git commit message work session 2](/admin/workflow-wiki/img/git_commit_message_2.png)
          Using `git log` I now see my two commits based off of `upstream/main`.
          <br/>
          ![git log 2](/admin/workflow-wiki/img/git_log_2.png)

    * Third work session
        - I chose not to `git pull --rebase` this time because I know there are
          no commits that may have been merged into `upstream/main` that I care
          about. To be safe, though, you can always `git pull --rebase` (as mom
          always said, it's better to have `pull rebase`d and not need it than
          need it and not have `pull rebase`d).
        - *Make the actual changes to code.*
        - *Test the code changes.*
        - Again, I stage, commit, and push my changes.
          <br/>
          ![git add commit push 2](/admin/workflow-wiki/img/git_add_cm_push_2.png)
        - Using `git log` I now see my three commits based off of
          `upstream/main`.
          <br/>
          ![git log 3](/admin/workflow-wiki/img/git_log_3.png)

    * Fourth work session
        - I chose not to `git pull --rebase` this time because I know there are
          no commits that may have been merged into `upstream/main` that I care
          about. To be safe, though, you can always `git pull --rebase`.
        - *Make the actual changes to code.*
        - *Test the code changes.*
        - Again, I stage, commit, and push my changes.
          <br/>
          ![git add commit push 3](/admin/workflow-wiki/img/git_add_cm_push_3.png)
        - Using `git log` I now see my three commits based off of
          `upstream/main`.
          <br/>
          ![git log 4](/admin/workflow-wiki/img/git_log_4.png)

3. Now that my changes are finished, I want to `squash` them into one commit
   with a better commit message. I do this through an interactive rebase
   (`git commit -i`).
    * First, I use `git log` to determine how many commits I want to `squash`.
      <br/>
      ![git log 5](/admin/workflow-wiki/img/git_log_5.png)
    * I see that there are 4 commits. So now I start the interactive rebase with
      those 4 commits. I will also include a fifth one just so I can make sure I
      went back as far as I intended but I will leave the fifth one alone. To do
      this, I type `git rebase -i HEAD~5`. This says go back as far as 5 commits
      before `HEAD` (which you can find in the `git log` output). Notice how the
      commits are in the reverse order as compared to `git log`.
      <br/>
      ![git interactive rebase 1](/admin/workflow-wiki/img/git_interactive_rebase_1.png)
    * My goal is to create 2 commits from these 5; I want to squash my 4 commits
      to 1 and leave the older commit as is. Thus, I want to squash my most
      recent 3 into the fourth oldest (the first one I worked on for the current
      issue I’m resolving). I `squash` these three by replacing the word `pick`
      with the letter `s` (short for `squash`) or the word `squash`. I’ve shown
      both here for the purpose of demonstation but you can just use one of the
      two.
      <br/>
      TODO: FIXME image is missing
      ![git interactive rebase 2](/admin/workflow-wiki/img/git_interactive_rebase_2.png)
    * Then I save and exit this file. Now, I am presented a list of the first
      (and only in our case) group commits I want to `squash` together.
      <br/>
      ![git interactive rebase 3](/admin/workflow-wiki/img/git_interactive_rebase_3.png)
    * I am going to comment out all of the commit messages and create an
      entirely new commit message. Note that I include `resolves #395` to link
      this commit to the issue I was working on. When I finish, I save and exit.
      <br/>
      ![git interactive rebase 4](/admin/workflow-wiki/img/git_interactive_rebase_4.png)
      <br/>
      ![git interactive rebase 5](/admin/workflow-wiki/img/git_interactive_rebase_5.png)
    * Using `git log` now, we can see that there is only one commit where there
      used to be four.
      <br/>
      ![git log 6](/admin/workflow-wiki/img/git_log_6.png)
    * I `git pull --rebase` to make sure I am up to date.
      <br/>
      ![git pull rebase 1](/admin/workflow-wiki/img/git_pull_rebase_1.png)
    * *Test the code changes to verify that nothing was broken accidentally in*
      *the interactive rebase.*
    * Since I changed the commit history (through the squashes in the
      interactive rebase), I will now have to `force push` to my branch. Be
      careful `force push`ing. If you’re hesitant, you can instead push to a
      new branch so that you can easily go back to before the rebase and try
      again. It's better to be safe than sorry.
      <br/>
      ![git force push 1](/admin/workflow-wiki/img/git_force_push_1.png)

## Merging Code Changes into Main

### Merging Code Changes into Main - Assumptions

### Merging Code Changes into Main - Details

### Merging Code Changes into Main - Example

