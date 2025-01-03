# Examples
List of practical examples for some of our workflows
Note that most of the examples will use the CLI for git, if you use an IDE like vscode you can also handle source control there. It doesn't make a difference to us.

## Writing a commit message
Let's say I have just added a feature to handle IMU sensor data, along with it I wrote some testcases. 
For the relevant feature code I would do this:
```
git add <relevant feature files>
git commit -m "feature(sensors): Store IMU sensor input to disk #0"
git add <relevant test files>
git commit -m "tests(sensors): Add testcases for handling IMU sensor input #0"
git push
```

## How to (interactively) rebase
### How to normally rebase
Let's say I have developed a bug fix for control systems, but in the meantime the main repo has had changes added to the sensor module so my feature branch and upstream main branch are out of date. A rebase can be used to update my feature branch, while ensuring that your new commits remain on top.

```
I have 1 commit on my branch with a control systems bug fix
git checkout main
git fetch upstream
git merge upstream/main
git checkout fix-branch
git rebase main
```
If there are conflicts, you would handle that after `git rebase main`

### How to interactively Rebase
Let's say I have 3 new commits on my branch:
- added tests
- fixed bug from last commit
- feat(control-systems): add PID control for knee joint motors #2  
 
The third commit is good, but the other 2 need some work, we want to squash the second commit into the 3rd one since this is a new feature so it doesn't make sense to push the feature and then immediately a bugfix, rather than putting them together. We will also need to reword the first commit to more clear wording.

```
git rebase -i HEAD~3 # -i flag specifies interactive, and HEAD~3 specifies the last 3 commits from HEAD
you will now see a list of 3 commits, and a set of options in your default text editor, (nano or vim most likely)
pick a0815dc added tests
pick <other SHA> fixed bug from last commit
pick <other SHA> feat(control-systems): add PID control for knee joint motors #2

to fix the first commit change 'pick' to r or reword, to change the second 2, change 'pic'k on the third commit to s or squash. Save these changes, it will open a new text editor where you can directly modify the commit message. For the second two there will be 2 commit messages, if you don't change anything it will combine them. Delete the line saying 'fixed bug from last commit' and save it

git log
this should now show only 2 commits on top of main, with the correct wording
```
If you have already pushed this feature branch to github, then you will have to force push these changes in order for it to apply remotely as well: `git push --force` (be cautious when doing this as it can overwrite things if done incorrectly)

If this is still all local, you can push as normal

