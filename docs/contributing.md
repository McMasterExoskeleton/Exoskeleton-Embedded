# How To Contribute to this Repo
We will be following a common workflow called the 'forking workflow' which is 
based on this: https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow

## Fork the main repository to create your own copy
This can be done through github, go to the the main repo and click 'fork' then specify repo name for your own copy. 
## Clone your forked repository 
```
git clone {repo_url}
```
## Add the main repository as a remote to the cloned repo:
`git remote add upstream {main_repo_url}` - this adds a remote repository called 'upstream'
You can verify that it worked by running `git remote -v`. There will likely be 4 entries, 2 called origin and 2 whatever you named the main repository (upstream, in this example). Each of the 2 corresponds to push or fetch.
## Developing a new feature
The idea is to create feature branches from your clone, where each branch is a standalone feature, you want to keep your main branch in sync with the main branch of the master repo. Once your branch is ready to be merged, create a pull request to the master repo and specify your branch as the target (Go to master repo on github -> pull requests -> new pull request -> select desired branch to be merged)
### Keeping your fork in sync with Master Repo
```
git fetch upstream
git checkout main
git merge upstream/main
```
### Creating a feature branch
```
git checkout main
git checkout -b new-feature
```
this creates and automatically checks out a branch called new-feature
### Push a branch to repo
The first time you are pushing your changes to a branch you have to publish the branch to your git repo
`git push -u origin new-feature` where new-feature is the branch name.
After this, you can push or pull to this branch like normal (if you use vscode source control, this is what 'publish branch' does)

## Commit Guide
In general, we aren't going to be too strict on commit messages, especially on your local branch - but depending on the size of the pull request we probably want it cut to 1-3 relevant commits. You may be required to reword or rebase commits if changes are needed, you can view [this example](examples.md#how-to-interactively-rebase) for some guidance. Commit messages should generally follow this standard:
```
type(module): brief description (should be imperative: ie Add feature X, instead of Added feature X)
Use additional lines if a more in-depth explanation is desired
Link the relevant github issue with #<issue_number>
```
The types we will follow are:
- fix: fixes a bug
- feature: something completely new to the codebase
- improvement: improvement to an existing feature
- test: modifies / adds testcases
- build: affects CI/CD pipeline or build system
- refactor: doesn't change code functionality
- docs: makes a change to documentation

The main modules for this repo are:
- sensors
- control_systems
- motors  

For a full example see [here](examples.md#writing-a-commit-message)






