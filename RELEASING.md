# Releasing Copper

## Branches and versions

```text
Development:
  master -> 1.3.0-dev

Release:
  tag v1.2.0
  create release/1.2

After release:
  master        -> 1.3.0-dev
  release/1.2 -> 1.2.1-dev

Patch release:
  release/1.2 -> 1.2.1
  tag v1.2.1
  then bump branch -> 1.2.2-dev
```

Development versions are not published or tagged. Release tags use
`vMAJOR.MINOR.PATCH`, and maintenance branches use `release/MAJOR.MINOR`.

Fixes normally land on `master` first and are then cherry-picked into each
supported release branch.

## Preparing 1.2

1. Audit publishable workspace members, including new crates, for package
   metadata, README and license files, packaged build inputs, and versioned
   registry dependencies. Optional dependencies also need registry versions.
2. Land publication fixes before cutting `release/1.2` from `master`.
3. Open `release-prep/1.2/version-bump` against `release/1.2`, changing the
   workspace version and internal Copper dependency requirements to `1.2.0`.
   Include standalone example, benchmark, and support manifests.
4. Run the release checks and verify package archives and dependency publication
   order. Run `just publish` from the approved release commit, then tag that
   commit `v1.2.0` and create the GitHub release.
5. Open a separate `chore/master-1.3.0-dev` PR against `master` to change Cargo
   manifests to `1.3.0-dev`. Keep this bump separate from release preparation.

The 1.1 precedent is PR #1264, targeting `release/1.1`, followed by PR #1267,
targeting `master`.
