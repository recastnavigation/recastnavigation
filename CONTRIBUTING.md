# Contribution Guidelines

We'd love for you to contribute to Recast and Detour and help make them even better than they are today! Here are the guidelines we'd like you to follow:

## Code of Conduct

This project adheres to the [Contributor Covenant Code of Conduct][code-of-conduct].  By participating in the Recast community, you are required to adhere to this code.

## Have a Question or Problem?

Questions about how to best use Recast, what it's capable of, and other inquiries should be directed to the [Q&A section of Github Discussions][q-and-a].  Questions submitted as Github issues will be converted to discussions.  

You can also submit questions to the older Recast [Google Group][groups] discussion list, or chat on [Gitter][gitter].

## Want a New Feature?

Check out our [development roadmap](Docs/_99_Roadmap.md) to see what new features are already planned.  The roadmap is a great resource if you're looking to help out with Recast but aren't sure where to start.

You can request a new feature by submitting a github issue.  See below for guidelines on submitting issues.

If you would like to implement a new feature then consider what category of change it is:
* **Major Changes** that you wish to contribute to the project should be discussed first in a [GitHub Issue][github-issues] or in our [Google Group][groups] so that we can coordinate efforts, prevent duplication of work, and help you to craft the change so that it is successfully accepted.
* **Small Changes** can be submitted as pull requests on [GitHub][github].

## Found a Bug?

If you've found a bug or an issue with the documentation you can help us by submitting an issue to our [GitHub Repository][github]. Even better you can submit a Pull Request with a fix.

### Submitting an Issue

Before you submit your issue search the [GitHub Issues][github-issues] and [Github Discussions][q-and-a] archives, maybe your question was already answered.

If your issue is a bug and hasn't been reported, open a new issue. Providing the following information will increase the chances of your issue being dealt with quickly:

* **Overview of the Issue** - what type of issue is it, and why is it an issue for you?
* **Callstack** - if it's a crash or other runtime error, a callstack will help diagnosis
* **Screenshots** - for navmesh generation problems, a picture really is worth a thousand words. Implement `duDebugDraw` and call some methods from DetourDebugDraw.h. Seriously, just do it, we'll definitely ask you to if you haven't!
* **Logs** - stdout and stderr from the console, or log files if there are any.
    If integrating into your own codebase, be sure to implement the log callbacks in `rcContext`.
* **Reproduction steps** - a minimal, unambiguous set of steps including input, that causes the error for you. e.g. input geometry and settings you can use to input into RecastDemo to get it to fail.
	  * Note: In RecastDemo you can save the intput parameters to Recast by pressing the `9` key.  The resulting `.gset` file can be shared with the `.obj` (if it is not one of the default ones) which will help tremendously in getting your bug repro'd and fixed.
* **Recast version(s) and/or git commit hashes** - particularly if you can find the point at which the error first started happening
* **Environment** - operating system, compiler etc.
* **Related issues** - have similar issues been reported before?
* **Suggest a Fix** - if you can't fix the bug yourself, perhaps you can point to what might be causing the problem (line of code or commit)

Here is a great example of a well defined issue: https://github.com/recastnavigation/recastnavigation/issues/12

**If you get help, help others. Good karma rulez!**

### Submitting a Pull Request

Check out [Github's official documentation](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) on how best to crete, work with and submit pull requests.

Before you submit your pull request consider the following:

* Search [GitHub Pull Requests][github-pulls] for an open or closed Pull Request that relates to your submission. You don't want to duplicate effort.
* Do your best to factor commits appropriately.  It's better to submit multiple pull separate requests than include every change in a single one. For example, if you're cleaning up some code and submitting a bug fix, it's best to submit a PR for the cleanup work separately from the bugfix.  This helps ensure your PR's are reviewed and merged quickly, and that an issue with some part of your changes doesn't hold back all of them.
* If applicable, please try to include some unit tests that test the issue and fix that you're submitting.
* Ensure your PR is rebased onto the head of `main`.  If you don't do this, you'll have some issues when github does it for 
you.

### Commit Message Format
Please format PR messages as follows (based on this [excellent post](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html)):

```
Summarize change in 50 characters or less

Provide more detail after the first line. Leave one blank line below the
summary and wrap all lines at 72 characters or less.

If the change fixes an issue, leave another blank line after the final
paragraph and indicate which issue is fixed in the specific format
below.

Fixes #42
```

Important things you should try to include in PR messages include:
* Motivation for the change
* Differences from previous behaviour
* Whether the change alters the public API, or meaningfully affects existing behaviour

[code-of-conduct]: https://github.com/recastnavigation/recastnavigation/blob/main/CODE_OF_CONDUCT.md
[q-and-a]: https://github.com/recastnavigation/recastnavigation/discussions/categories/q-a
[github]: https://github.com/recastnavigation/recastnavigation
[github-issues]: https://github.com/recastnavigation/recastnavigation/issues
[github-pulls]: https://github.com/recastnavigation/recastnavigation/pulls
[gitter]: https://gitter.im/recastnavigation/chat
[groups]: https://groups.google.com/forum/?fromgroups#!forum/recastnavigation
