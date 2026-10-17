# Polynomials Trajectory Planner

Educational + practical project: analytical generation and analysis of polynomial trajectory profiles (degrees 3, 5, 7, 9, extensible), culminating in a C#/WPF desktop application and a personal theory/knowledge library.

Development is carried out exclusively by Claude (Cowork for specification, theory and roadmap; Claude Code for the application code), coordinated through the files in `coordination/`. See `CLAUDE.md` for the working rules.

## Repository structure

```
spec/           master specification (vision, requirements, data model, assumptions, future topics, success criteria)
theory/         theory library: one document per requirement cluster (symbols, degree-9 theory, boundary conditions, ...)
design/         implementation plan and UI specification (bridge between theory and code)
roadmap/        ROADMAP.md (milestones and priorities), theory library roadmap, decisions/ (one note per decision)
coordination/   Cowork <-> Code channel: STATUS.md, inbox-code/ (requests), inbox-docs/ (return notes)
src/            C#/WPF application: Core (calculation), App (WPF), Core.Tests (xUnit)
assets/         icons and graphics
archive/        original brief and superseded generated documents
build/          generated PDFs (not tracked)
LearningCsWPF/  separate learning material, out of scope for now
```

## Where to start

- `spec/00-vision.md` … `spec/05-success-criteria.md` are the **master specification**: read-only reference, amended only deliberately when a requirement genuinely changes.
- `roadmap/06-theory-library-roadmap.md` is the checklist and format convention for the theory library (`theory/`).
- `design/07-implementation-plan.md` records the implementation decisions; `design/08-ui-first-prototype.md` specifies the first GUI milestone.
- `roadmap/ROADMAP.md` and `coordination/STATUS.md` give the current state and next steps.

## Build and test

```
dotnet build PolynomialsTrajectoryPlanner.slnx
dotnet test  src/TrajectoryPlanner.Core.Tests
```

The App project targets `net8.0-windows` (WPF) and runs on Windows only.
