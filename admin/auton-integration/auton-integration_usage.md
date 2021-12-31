## Motivation

Auton_integration is designed to be an integration testing branch that all code is tested in prior to being merged in.

It is common for the code of other subteam branches to have discrepancies between main and auton_integration. What commonly happens is people get their code ready to be merged into main, but then have to merge it into auton_integration and either have to make a new branch with their changes based off auton_integration or they will change the files belonging to another subteam when merging changes in. Additionally, it is annoying when members of the same subteam merge multiple pull requests into auton_integration and have to resolve merge conflicts when doing so. Then they’ll have to resolve the same merge conflicts when merging into main, which is doing the same work twice. 

The process described below allows sub-teams to update and test on auton_integration without having to worry if other sub-team’s code is up to date. Below is an example of the ideal auton_integration branch. Each sub-team’s code is at a base level up to date with main, but there are individual commits applied to auton_integration that only affect a single subteam. No commit affects the code of more than one subteam. The use of codeOwners can be leveraged to ensure that no other subteam PRs in a commit that affects your subteam’s code without the approval of the lead of that subteam.

    c5
    c4				    c6
    c1		    c2		c3
    main		main	not up to date with main
    percep    	nav		loc


## Pushing Code to auton_integration

Note: This process requires the code that is owned by your subteam on auton_integration to be up to date with the code that is owned by your sub-team in main

* Create a branch M that contains your code and is able to be PR’d into main
* Create a new branch K and reset this branch to be exactly like main
* Jump onto branch K
*       git merge --squash M
* Add, commit, and push these changes
* The commit you just pushed is commit C
* This commit contains all of the changes that you have made
* Create another branch J that is the same as auton_integration
* git cherry-pick C onto J
* PR J into auton_integration
    * You may have merge conflicts to resolve
* After testing, to prevent you from having to resolve those conflicts again, git cherry-pick your PR commit from auton_integration onto your branch that is going to be PR’d into main
    * This may cause conflicts; however, you can confidently accept all incoming changes, since this will make your branch that is being PR’d into main the same as the resolved one on auton_integration
* The branch you cherry-picked onto should now be all integration tested and ready for a PR into main
    * Be sure to provide a link in this PR to your PR into auton_integration
    * This assumes that PRs are getting merged into main in the same order that they were added to auton_integration

This process is automated by:

    ./pr_auton_integration
## Get auton_integration up to date with main

Note: This should be done once tested commits have been merged into main
Note: This is essential if the previous process is to be successful

* Create a branch that is up to date with auton_integration
* Enter into the mrover-workspace directory
*       git fetch upstream
*       git checkout upstream/main jetson/<sub-team folder>
* Let remote upstream = https://github.com/umrover/mrover-workspace 
* Your sub-team folder should now have been replaced with your sub-team folder from main

This process is automated by:

    ./pr_auton_integration update
## Git cherry-pick

Git cherry-pick allows you to grab a specific commit and apply it to another branch without having to merge in all of the code from one branch into the other. This is nice since other subteams can have their branches in any state that they want and your code will not be affected by the state of their branch. 

* Git cherry-pick commit C onto branch M:
* Find the hash H of commit C (can be seen next to the word “commit” when running git log)
* checkout branch M
* git cherry-pick H
