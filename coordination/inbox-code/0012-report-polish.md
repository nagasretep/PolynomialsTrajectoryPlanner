# 0012 — Report polish: blend adjustment notice, one output culture

Status: closed · From: Cowork · To: Code · Date: 2026-10-17

Run after request 0011.

## Goal

Two small user-visible points from the review of 0010: (1) when limits force a blend duration different from the inherited one, the user is told; (2) numbers are displayed in a single culture across the App.

## Reference documents

- `theory/BlendingSegments.md`, *Decisions For The 1D Baseline* (bullet "Visibility")
- `roadmap/discrepancies.md`: D21, D22; note `coordination/inbox-docs/0010-blend-duration-and-parsing.md`

## Scope

- In scope:
  1. **Core**: the blend result exposes the inherited duration, the chosen duration and whether the chosen one was adjusted because of the limits (a small value on the result, or properties on the blend; your choice, keep it simple). No change to the computed durations.
  2. **Service/UI**: when a blend was adjusted, add a line to the report and/or the status bar: "Blend 1-2: duration adjusted from X s to Y s to respect the limits" (X, Y formatted like the other durations). No message when nothing was adjusted.
  3. **Output culture (D21)**: all displayed numbers use the current culture (XAML tables, status bar, cursor). Set the WPF default language to the current culture at start-up (`FrameworkElement.LanguageProperty.OverrideMetadata` or equivalent) and check that the XAML bindings follow it. If something cannot follow without a larger change, report it in the note. Inputs are already accepted in both forms (0010).
  4. Tests: service-level test for the adjustment message (adjusted and not adjusted); a culture test for the formatting helper if there is one; if the XAML output cannot be tested without a window, say how you checked it.
- Out of scope: any change to the durations themselves, R9–R11, other UI changes.

## Acceptance criteria

- [ ] An adjusted blend produces the message with the right values; a non-adjusted blend produces none.
- [ ] On an `it-IT` setup the tables, status bar and cursor show the same decimal separator (verified as far as possible without the window; tell the author what to look at when running the App).
- [ ] `dotnet build` free of new warnings; all tests pass; the note states the test count.

## Expected tests

- Blend example of 0009/0010 with `v_max = 8`: not adjusted (inherited 0.643 s feasible) → no message; tighter limit (inherited infeasible) → message with both durations.
- Formatting under `it-IT` and `en-US` set explicitly (restored afterwards).

## Notes for the author (educational)

Explain in the return note how WPF chooses the culture for bindings (the `Language` property of the element, default en-US, independent of the Windows setting) and why that produced a mix of formats.

Return note: `coordination/inbox-docs/0012-report-polish.md`, ending with a "Suggested commits" section.
