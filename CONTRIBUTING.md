# Contribution Guidelines

This page provides general guidelines to contribute efficiently to the project HSR project.

**Content:**

*   [Project Language](#project-language)
*   [User Configuration](#user-configuration)
*   [Git Workflow](#git-workflow)
*   [Protected Branches](#protected-branches)
*   [Branch Names](#branch-names)
*   [Commit Messages](#commit-messages)
*   [File Names](#file-names)

## Project Language

Commit messages should be written in English. Elsewhere, English is desired and Japanese is tolerated. However, messages in other languages will be ignored and/or removed.

## User Configuration

Make sure to use your own Git settings before committing or pushing for the first time, especially on shared machines:

```shell
git config --global --unset user.name
git config --global --unset user.email
cd ~/HSR/ && git config user.name "Firstname Lastname"
cd ~/HSR/ && git config user.email "address@example.com"
```

## Git Workflow

1.   Pull the latest version of the remote `devel` branch on your local machine.
2.   Create a new local branch to work with from the pulled `devel` branch on your local machine.
3.   Work in the newly created local branch until your code well tested and documented.
4.   Pull once more the latest version of the remote `devel` branch and merge it to your local branch on your machine.
5.   Push the merged local branch to the remote server and create a merge request from the pushed branch to the remote `devel` branch.

## Protected Branches

*   `devel`: Default repository branch used for main development. Developers can push code but only Maintainers can merge.
*   `master`: Branch used for deployment and release of stable versions. No one can push code and only Maintainers can merge.

## Branch Names

*   Do not use any capital letter or punctuation.
*   Always use hyphens `-` instead of underscores `_`.
*   Always try to start with either `add-`, `set-`, `fix-`, or `try-`. If not possible, use another verb in imperative mood.

## Commit Messages

*   Use English language only.
*   Always start with a verb in imperative mood.
*   Capitalize the first letter.
*   Do not use punctuation at the end.
*   Add the prefix `WIP: ` for untested code.
*   Put code-specific names inside single quotation marks `' '`.

Please read the famous post *"[How to Write a Git Commit Message](https://chris.beams.io/posts/git-commit/)"* by Chris Beams.

## File Names

*   Always prefer hyphens `-` to underscores `_` except when required by convention, such as in ROS.
*   Use prefixes to distinguish first-party resources from third-party ones. For example, add the prefix `sdewg_` to the names of the Software Development Environment Working Group (SDE-WG) packages, nodes, topics, and services. Other prefixes currently in use are: `hsr_` (commons) and `3rd_` (third-parties).
