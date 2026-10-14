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

## Independent patch releases

All first-party registry dependencies use minor-line requirements (`~1.3.0`
for Copper and `~2.2.0` for the independently versioned codec crates), including
optional, build, target-specific, and standalone-manifest dependencies. Keep
package versions exact. Generated projects follow the same dependency policy.

For a patch release, bump only the packages that changed. Override
`version.workspace = true` with the changed package's exact patch version when
necessary; leave the workspace version and unchanged packages alone. Retain each
dependency's existing minimum unless the change actually requires a newer API or
fix. Raise that minimum only for the affected dependency, keeping its `~` range.

Run the release checks, then `just publish` to publish only versions absent from
the registry. Do not globally bump dependencies or republish unchanged crates to
release one package's patch. The codec and derive crates can also receive patches
independently under their `~2.2.0` requirements.

## GitHub release checklist

Complete this checklist for every stable minor or patch release, including
backports to supported maintenance branches.

1. Update the version's entry in the global
   [Copper Release Notes](https://github.com/copper-project/copper-rs/wiki/Copper-Release-Notes).
   Include upgrade instructions and compatibility warnings where needed.
2. Push `vMAJOR.MINOR.PATCH` on the exact commit used to publish the crates.
   Verify the commit's manifests and release changes before tagging it.
3. Copy that version's complete notes into a Markdown file inside the worktree.
   Preserve reference-link definitions, images, and compatibility warnings.
   Shared notes for multiple versions belong in each corresponding release.
4. Create or update the GitHub Release for that tag using those notes. Publish
   an existing draft when one is already present. Mark only the newest stable
   version as Latest; use `--latest=false` for historical releases and backports.
5. Verify that the entry is published, its tag resolves to the intended commit,
   its notes match the global notes, and Latest still points to the newest
   stable release.

For example, after pushing an existing historical tag:

```bash
gh release create v1.1.4 --repo copper-project/copper-rs \
  --verify-tag --title "Copper 1.1.4" \
  --notes-file target/release-notes/v1.1.4.md --latest=false
```

For an existing draft, use `gh release edit` with `--draft=false`,
`--notes-file`, and the appropriate `--latest` setting.

When filling historical gaps, reuse existing tags. If a tag is missing, identify
the original release commit from Git history and verify its version and changes
before creating the tag. The global notes are the source for the release body;
retain their original release date in the body when publishing an entry later.
