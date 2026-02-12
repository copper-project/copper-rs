# Copper Workspace README Style Guide

This document defines a unified README format for crates in `core/`, `components/`, and `examples/`.

## Goals

- Keep crate documentation scannable and consistent.
- Make first-run and integration steps obvious.
- Standardize required headings so CI can enforce basic quality.

## Global Rules

- Use a single H1 title as the first heading: `# <name>`.
- Keep required sections in the exact order defined by crate type.
- Prefer concise prose and runnable command snippets.
- Use `ron` code blocks for Copper config snippets.
- Use relative links for in-repo references.
- Put unresolved or extra details under `## Additional Notes`.

## Core Crate README Template

Required headings in order:

1. `## Overview`
2. `## Usage`
3. `## Compatibility`
4. `## Links`

Optional headings:

- `## Feature Flags`
- `## Examples`
- `## Performance Notes`
- `## Limitations`
- `## Additional Notes`

## Component Crate README Template

Required headings in order:

1. `## Overview`
2. `## Interface`
3. `## Configuration`
4. `## Usage`
5. `## Compatibility`
6. `## Links`

Optional headings:

- `## Hardware Notes`
- `## Payloads`
- `## Troubleshooting`
- `## Limitations`
- `## Additional Notes`

Notes:

- `## Interface` should state data flow shape (input/output/channels/resources).
- `## Configuration` should list keys with short meanings and defaults if available.

## Example Crate README Template

Required headings in order:

1. `## Overview`
2. `## Prerequisites`
3. `## Run`
4. `## Expected Output`
5. `## Links`

Optional headings:

- `## Architecture`
- `## Configuration`
- `## Troubleshooting`
- `## Next Steps`
- `## Additional Notes`

## Link Conventions

- Include at least one useful link under `## Links`:
  - local path to source/config/docs
  - docs.rs page (for library crates)
  - related wiki page when relevant

## Lint Enforcement

README structure is validated by `support/ci/readme_lint.py`.

Run locally:

```bash
just readme-lint
```
