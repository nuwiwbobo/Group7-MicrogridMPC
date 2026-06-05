# Contributing to Group7-MicrogridMPC

Thanks for collaborating! This is a small, time-boxed project (deadline 2026-06-10). Keep the rules minimal so we can move fast.

## Branching

- **Default branch:** `main` (protected informally — no force-pushes)
- **Feature branches:** `feat/<your-name>-<short-desc>` (e.g. `feat/kholifah-plant-model`)
- **Bugfix branches:** `fix/<short-desc>` (e.g. `fix-qp-infeasibility`)
- **Direct commits to `main`:** OK for small stuff (typos, README updates, 1-line fixes)

## Commit messages

Use the format: `area: imperative summary`

Examples:
- `plant: add plant_model.m with A/B/C matrices`
- `sim: closed-loop driver for 7-day simulation`
- `report: add section 3 prediction matrices`
- `fix: handle quadprog exit code -2 (infeasible)`

Keep the summary under 72 characters. Add a blank line + body for non-obvious changes.

## Pull requests

- **Required for:** new functions, new modules, >50 lines of new code
- **Skip PR for:** small docs/typo/config changes
- **At least one teammate should review** before merge (use GitHub's "request review" feature)

## Hand-off contract

If you are writing a function that another teammate will call:
1. Document the inputs and outputs at the top of your .m file
2. Add a usage example in the comments
3. Update `README.txt` if it's a new top-level entry point
4. Mention the change in the team chat

## MATLAB style

- One function per file; filename matches the function name
- `%` line comments for top-of-file docs, `%` inline for clarity
- `snake_case` for filenames, `lowerCamelCase` for local variables
- 4-space indentation
- Avoid global state; pass everything as function arguments

## What NOT to commit

- Personal debug scripts (use a `scratch/` folder outside the repo)
- Large binary files (figures, datasets) — use `images/` and `data/` and reference by path
- Generated PDFs from LaTeX (the `.gitignore` blocks these; regenerate from `.tex` if needed)
- IDE settings, OS metadata (already in `.gitignore`)

## Questions?

Ping Kholifah (owner) or open a Discussion on the GitHub repo.
