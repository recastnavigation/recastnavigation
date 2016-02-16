# Contributing to Recast and Detour

We'd love for you to contribute to our source code and to make Recast and Detour even better than they are
today! Here are the guidelines we'd like you to follow:

 - [Code of Conduct](#coc)
 - [Question or Problem?](#question)
 - [Issues and Bugs](#issue)
 - [Feature Requests](#feature)
 - [Submission Guidelines](#submission-guidelines)
 - [Git Commit Guidelines](#git-commit-guidelines)

## <a name="coc"></a> Code of Conduct
This project adheres to the [Open Code of Conduct][code-of-conduct].
By participating, you are expected to honor this code.

## <a name="question"></a> Got a Question or Problem?

If you have questions about how to use Recast or Detour, please direct these to the [Google Group][groups]
discussion list. We are also available on [Gitter][gitter].

## <a name="issue"></a> Found an Issue?
If you find a bug in the source code or a mistake in the documentation, you can help us by
submitting an issue to our [GitHub Repository][github]. Even better you can submit a Pull Request
with a fix.

**Please see the Submission Guidelines below**.

## <a name="feature"></a> Want a Feature?
You can request a new feature by submitting an issue to our [GitHub Repository][github]. If you
would like to implement a new feature then consider what kind of change it is:

* **Major Changes** that you wish to contribute to the project should be discussed first on our
[Google Group][groups] or in [GitHub Issues][github-issues] so that we can better coordinate our efforts, prevent
duplication of work, and help you to craft the change so that it is successfully accepted into the
project.
* **Small Changes** can be crafted and submitted to the [GitHub Repository][github] as a Pull Request.

## Submission Guidelines

### Submitting an Issue
Before you submit your issue search the [GitHub Issues][github-issues] archive,
maybe your question was already answered.

If your issue appears to be a bug, and hasn't been reported, open a new issue.
Help us to maximize the effort we can spend fixing issues and adding new
features, by not reporting duplicate issues. Providing the following information will increase the
chances of your issue being dealt with quickly:

* **Overview of the Issue** - what type of issue is it, and why is it an issue for you?
* **Callstack** - if it's a crash or other runtime error, a callstack will help diagnosis
* **Screenshots** - for navmesh generation problems, a picture really is worth a thousand words.
    Implement `duDebugDraw` and call some methods from DetourDebugDraw.h. Seriously, just do it, we'll definitely ask you to if you haven't!
* **Logs** - stdout and stderr from the console, or log files if there are any.
    If integrating into your own codebase, be sure to implement the log callbacks in `rcContext`.
* **Reproduction steps** - a minimal, unambigious set of steps including input, that causes the error for you.
    e.g. input geometry and settings you can use to input into RecastDemo to get it to fail.
	Note: These can be saved by pressing the 9 key in RecastDemo, and the resulting .gset file can be shared (with the .obj if it is not one of the default ones).
* **Recast version(s) and/or git commit hash** - particularly if you can find the point at which the error first started happening
* **Environment** - operating system, compiler etc.
* **Related issues** - has a similar issue been reported before?
* **Suggest a Fix** - if you can't fix the bug yourself, perhaps you can point to what might be
  causing the problem (line of code or commit)

Here is a great example of a well defined issue: https://github.com/recastnavigation/recastnavigation/issues/12

**If you get help, help others. Good karma rulez!**

### Submitting a Pull Request
Before you submit your pull request consider the following guidelines:

* Search [GitHub Pull Requests][github-pulls] for an open or closed Pull Request
  that relates to your submission. You don't want to duplicate effort.
* Make your changes in a new git branch:

     ```shell
     git checkout -b my-fix-branch master
     ```

* Implement your changes, **including appropriate tests if appropriate/possible**.
* Commit your changes using a descriptive commit message that follows our
  [commit message conventions](#commit-message-format).

     ```shell
     git commit -a
     ```
  Note: the optional commit `-a` command line option will automatically "add" and "rm" edited files.

* Squash any work-in-progress commits (by rebasing) to form a series of commits that make sense individually.
  Ideally the pull request will be small and focused enough that it fits sensibly in one commit.

     ```shell
     git rebase -i origin/master
     ```

* Push your branch to GitHub:

    ```shell
    git push origin my-fix-branch
    ```

* In GitHub, send a pull request to `recastnavigation:master`.
* If we suggest changes then:
  * Make the required updates.
  * Commit your changes to your branch (e.g. `my-fix-branch`).
  * Squash the changes, overwriting history in your fix branch - we don't want history to include incomplete work.
  * Push the changes to your GitHub repository (this will update your Pull Request).

If you have rebased to squash commits together, you will need to force push to update the PR:

    ```shell
    git rebase master -i
    git push origin my-fix-branch -f
    ```

That's it! Thank you for your contribution!

#### After your pull request is merged

After your pull request is merged, you can safely delete your branch and pull the changes
from the main (upstream) repository:

* Delete the remote branch on GitHub either through the GitHub web UI or your local shell as follows:

    ```shell
    git push origin --delete my-fix-branch
    ```

* Check out the master branch:

    ```shell
    git checkout master -f
    ```

* Delete the local branch:

    ```shell
    git branch -D my-fix-branch
    ```

* Update your master with the latest upstream version:

    ```shell
    git pull --ff upstream master
    ```

## Git Commit Guidelines

### Commit content

Do your best to factor commits appropriately, i.e not too large with unrelated
things in the same commit, and not too small with the same small change applied N
times in N different commits. If there was some accidental reformatting or whitespace
changes during the course of your commits, please rebase them away before submitting
the PR.

### Commit Message Format
Please format commit messages as follows (based on this [excellent post](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html)):

```
Summarize change in 50 characters or less

Provide more detail after the first line. Leave one blank line below the
summary and wrap all lines at 72 characters or less.

If the change fixes an issue, leave another blank line after the final
paragraph and indicate which issue is fixed in the specific format
below.

Fix #42
```

Important things you should try to include in commit messages include:
* Motivation for the change
* Difference from previous behaviour
* Whether the change alters the public API, or affects existing behaviour significantly



[code-of-conduct]: http://todogroup.org/opencodeofconduct/#Recastnavigation/b.hymers@gmail.com
[github]: https://github.com/recastnavigation/recastnavigation
[github-issues]: https://github.com/recastnavigation/recastnavigation/issues
[github-pulls]: https://github.com/recastnavigation/recastnavigation/pulls
[gitter]: https://gitter.im/recastnavigation/chat
[groups]: https://groups.google.com/forum/?fromgroups#!forum/recastnavigation
