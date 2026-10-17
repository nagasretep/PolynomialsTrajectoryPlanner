# CLAUDE.md — working rules for this repository

This project is developed exclusively by Claude, in two roles that share this repository. Any reference to Trae AI in older documents or code is obsolete and should be removed when the file is next touched.

Conversation with the author is in Italian; repository documents and code are in English.

## Roles and ownership

| Area | Owner | The other role may |
|---|---|---|
| `spec/`, `theory/`, `design/`, `roadmap/`, `README.md` | **Cowork** | read; propose changes via `coordination/inbox-docs/` |
| `src/`, `PolynomialsTrajectoryPlanner.slnx`, `assets/` | **Claude Code** | read; request changes via `coordination/inbox-code/` |
| `coordination/STATUS.md` | whoever closes a task | update |
| `.gitignore`, `CLAUDE.md` | Cowork, with the author's agreement | read |

Rule: one owner per file. Never edit a file owned by the other role; write a request or a note instead.

## Out of scope

`LearningCsWPF/` is ignored entirely for now. Do not read, modify or reorganize it.

## Coordination protocol

1. **Cowork → Code**: a request file in `coordination/inbox-code/`, named `NNNN-short-title.md` (template: `_TEMPLATE-request.md`). It states the goal, the theory documents to follow, acceptance criteria and expected tests.
2. **Code → Cowork**: when a task is done or blocked, a note in `coordination/inbox-docs/`, named after the request (`NNNN-short-title.md`, template: `_TEMPLATE-note.md`). It states what was done and why, any choice not covered by the theory, any discrepancy between theory and code, and open questions.
3. Update `coordination/STATUS.md` (current milestone, in progress / blocked / to review) when closing a task.
4. A processed request or note gets `Status: closed` at the top and stays in place; nothing is deleted.
5. Decisions that change design go to `roadmap/decisions/` (one file each, `ADR-NNNN-title.md`).

## The author stays involved

The author must remain aware of the content of the software (`spec/01-requirements.md`, item 16): educational value takes precedence over development speed. For every non-trivial change, Code writes a short "what and why" note so that Cowork can turn it into documentation.

## Code conventions

- `TrajectoryPlanner.Core` is a pure calculation library with no UI dependency; every theory document maps to one folder/namespace of `Core` (see `design/07-implementation-plan.md`).
- `TrajectoryPlanner.App` (WPF, ScottPlot) references `Core`; calculation logic does not live in the UI layer.
- `TrajectoryPlanner.Core.Tests` (xUnit) tests `Core` only.
- Theory comes first: if code and theory disagree, do not silently pick one — record it in a note for Cowork.
- Symbols follow `theory/SymbolsAndNomenclature.md` (polynomial coefficients are `c_i`, not `a_i`).
- Never commit `bin/`, `obj/`, `.vs/` or generated PDFs (see `.gitignore`).

## Documentation conventions (Cowork)

- Source format is Markdown with inline LaTeX math, as defined in `roadmap/06-theory-library-roadmap.md` (format convention and pre-publication checklist).
- PDFs are generated into `build/` with pandoc and are never edited by hand.
- `spec/` is amended only deliberately, never regenerated.

## Commits

Prefix the message by area: `docs:` (Cowork areas), `code:` (Code areas), `coord:` (coordination), `repo:` (structure, ignore rules). Keep documentation and code changes in separate commits.
