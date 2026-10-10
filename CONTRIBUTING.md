# Contributing

## Branches
- main: verified project milestones.
- integration: combine and check everyone's work.
- feature/<task-name>: one focused task.

Workflow: feature branch → integration → main.

Only Andrew merges integration into main.
Do not push directly to main or integration.

## Start a Task
1. Choose a task from V1_CHECKLIST.md and confirm ownership.
2. Update your local integration branch.
3. Create a feature branch from integration.

```bash
git switch integration
git pull --ff-only origin integration
git switch -c feature/your-task-name