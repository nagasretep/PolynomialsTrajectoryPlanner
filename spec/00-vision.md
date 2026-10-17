# Vision

The goal of this project is to enter in detail into the trajectory planning process, considering polynomial algorithms of different degree.

The degrees to be approached are, at the moment, three, five, seven and nine. Other degrees will be added over time if deemed useful during development (see `04-future-topics.md`).

The project will culminate in an **application** (web or desktop — see the open decision in `01-requirements.md`, item 16) that analytically determines the trajectories discussed throughout the project. It is also intended as a personal learning experience in polynomial calculus and the techniques behind it. Everything produced should therefore be as educational as possible.

## Outputs

Two outputs are expected:

1. **The application** — graphical output and data input, satisfying the technical specifications.
2. **A knowledge reference** — a readable document (or group of documents) / project log, capturing procedures, calculations, considerations, and theory as they are developed. Ideally this log records the steps taken and the theoretical/practical considerations behind each development decision.

## Topic classification

Every requirement in `01-requirements.md` is tagged as one of:

- `[Learning]` — theoretical concepts and procedures aimed at deepening understanding of the topic.
- `[Technical]` — aspects directly involved in the implementation of the application.
- `[Learning & Technical]` — applies to both.

For every tagged item, a written report of the procedures (calculations, considerations) and algorithms used — and, when applicable, of the underlying theory — is expected.

## General rules (apply throughout, unless a requirement says otherwise)

To avoid repeating the same clause on every requirement, the following rules are declared once here and apply globally:

- **Dimensionality**: every analysis, algorithm, or consideration is first developed in one-dimensional (1D) space. Once established, any differences arising when extending the same concept to three-dimensional (3D) space are noted explicitly. This progression (1D → note 3D differences) is the default for all requirements unless stated otherwise.
- **Numerical vs. symbolic**: unless numerical data is strictly necessary for the context, symbolic expressions are preferred over numerical examples.
