# ADR-0001 — Repository restructure and Claude-only development

Date: 2026-10-17 · Status: accepted by the author

## Context

The project grew as `project/docs` (specification, theory, design) plus `project/src`. Development was previously assisted by another tool (Trae AI); from now on it is carried out exclusively by Claude, in two environments: Cowork (specification, theory, roadmap) and Claude Code (application code). The repository also tracked build output (`bin/`, `obj/`, `.vs/`) with no `.gitignore`.

## Decision

- Reorganize the repository into `spec/`, `theory/`, `design/`, `roadmap/`, `coordination/`, `src/`, `assets/`, `archive/` (see `README.md`).
- One owner per area; cross-requests and return notes travel as Markdown files in `coordination/` (see `CLAUDE.md`).
- Add `.gitignore` and stop tracking `bin/`, `obj/`, `.vs/`; history is not rewritten.
- `LearningCsWPF/` is left untouched and out of scope.
- Trae AI references in existing documents are obsolete.

## Consequences

- File moves are done with `git mv`, so file history is preserved.
- The icon path in `TrajectoryPlanner.App.csproj` changes (`assets/`).
- Documents that mention `docs/` or `project/` paths need a reference pass (task for Cowork).
