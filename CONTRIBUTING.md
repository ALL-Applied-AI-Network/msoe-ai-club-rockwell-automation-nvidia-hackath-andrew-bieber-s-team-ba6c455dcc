# Contributing

## Branches and Reviews

- `main`: verified project milestones.
- `integration`: combine and check team contributions.
- `feature/<task-name>`: one focused task.

Workflow: **feature branch → pull request to integration → verified milestone → main**.
Only Andrew merges `integration` into `main`. Do not push work directly to `main`
or `integration`.

Before the team starts, Andrew must confirm that the shared `integration` branch
exists and is ready. If it is missing, ask Andrew to initialize it; do not substitute
`main` or create an unrelated local integration history.

## Start a Task

1. Confirm ownership and the deliverable in `V1_CHECKLIST.md`.
2. Check that you have no uncommitted work before switching branches.
3. Run from the repository root:

```bash
git status
git fetch origin
git switch integration
git pull --ff-only origin integration
git switch -c feature/your-task-name
```

If `integration` exists remotely but not locally, use
`git switch --track origin/integration` in place of `git switch integration`.
Replace `your-task-name` with a short descriptive name.

## Submit Work

```bash
git status
git add path/to/changed-file
git commit -m "Describe the change"
git push -u origin feature/your-task-name
```

Open a pull request with **base: integration**. Explain what changed, why, how to
run or inspect it, what was verified, and any remaining blockers. Link logs or
reports where appropriate. Use explicit paths with `git add` to avoid accidentally
including generated files.

## Working Between VS Code and Rosie

Commit and push changes from your local feature branch. On Rosie, fetch and switch
to that same branch, then pull with `--ff-only`. Check `git status` first on both
machines. Resolve conflicting edits; do not discard work just to make a pull succeed.

Use the team repository's `fanuc_er4ia_sim/` for development. A saved USD scene is
a snapshot; pulling Python changes does not regenerate it.

## Validation

Read `fanuc_er4ia_sim/AGENTS.md` and relevant starter documentation before changing
simulator code. Use host Python for host tests and Isaac Python for simulation.
Record which checks actually ran. Numerical IK or host tests do not prove a grasp
succeeded; robot motion alone does not prove object movement.

Preserve the full starter benchmark's inventory, perception, pickup, and placement
gates. Keep known-position diagnostic results separate from camera-based results.

## Files and Team Coordination

- Commit source, small configuration files, and documentation.
- Keep environments, credentials, model weights, datasets, generated textures,
  and run media out of Git; document approved data locations instead.
- Update your task row and link evidence before marking it complete.
- Raise blockers early with what you tried and what you need.
- Make physical-cell changes or execute commands only through the approved,
  supervised process.
