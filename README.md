# 2017-inseason
## Steamworks code of FRC Team 973: Greybots
#### Presenting our 2017 Houston World Championship Robot: [Bloodhound](https://www.youtube.com/watch?v=vJeKOW3ZpIk)

[![Build Status](https://travis-ci.com/Team973/2017-inseason.svg?token=PMQ4h4i9r3eRUJnsCJBt&branch=master)](https://travis-ci.com/Team973/2017-inseason)

## Structure

`src`: Actual robot code
`lib`: Code that is shared between all robots, Modified WPILib to suit our robot
`test`: Code used to run a test

## Building the Code

We use cmake to build our code. To get it, follow the instructions in the `greybots-skeleton` repo.
`cd repo-name/build`
`make`
To deploy to the robot, make sure you're connected to the robot and do
`make deploy`

## Contributing

Here's how to get your code into the main robot repository:

### If you've just joined the team:
1. Make an account on GitHub.
2. Ask the robot programming lead to add your account to the frc1678 robot programming team.
**If it's the first time you've contributed to this repo:**

3. Clone this repo
4. Log onto github and navigate to the repo 2017-inseason.

`git clone https://github.com/<your_name>/robot-code.git`, where <your_name> is your github username.

**Any time you want to make a change:**

Create and checkout a new branch.
`git checkout -b <your_branch_name>`, where <your_branch_name> is a descriptive name for your branch. 
Use dashes in the branch name, not underscores. 
**Do not create a new branch unless a same branch has already been made.**
Make whatever code changes you want/need to make. 
Be sure to write tests for your changes!
Commit your work locally.
Try to make your commits as small as possible. 
For example, moving functions around should be different from adding features, and changes to one subsystem should be in a different commit than changes to another subsystem.
Follow the conventions for commit messages.
If your change is anything more than a few lines or small fixes, don't skip the extended description. 

**To push to a specific branch of the repo.**
`git push origin <your_branch_name>`
Submit a pull request.
Log into github.
Go to the page for your forked repo.
Select the branch that you just pushed from the "Branch" dropdown menu.
Click "New Pull Request".
Review the changes that you made.
If you are happy with your changes, click "Create Pull Request".
People (mentors and programming lead) must review (and approve of) your changes before they are merged.
Specifically, the rules are that one of the following two conditions must be true for it to get merged:
1 mentor and 1 other person have approved
2 experienced students and one other person have approved
If there are any concerns about your pull request, fix them. 
To update your PR, just push to the branch on your forked repo.
Merge your changes into master

**If there are no conflicts, push the "Squash and merge" button, write a good commit message, and merge the changes.**

**If there are conflicts, fix them locally on your branch, push them, wait for Travis to pass, and then squash and merge.**

**Questions? Email jacks8211@gmail.com**
