# Releasing Copper

## Branches and versions

```text
Development:
  master -> 1.1.0-dev

Release:
  tag v1.1.0
  create release/v1.1

After release:
  master        -> 1.2.0-dev
  release/v1.1 -> 1.1.1-dev

Patch release:
  release/v1.1 -> 1.1.1
  tag v1.1.1
  then bump branch -> 1.1.2-dev
```

Development versions are not published or tagged. Release tags use
`vMAJOR.MINOR.PATCH`, and maintenance branches use `release/vMAJOR.MINOR`.

Fixes normally land on `master` first and are then cherry-picked into each
supported release branch.
