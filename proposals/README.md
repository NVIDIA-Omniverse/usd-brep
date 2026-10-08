# Vendored OpenUSD proposals

## UsdSolid

`UsdSolid/` and its linked `cad_geometry/` problem statement are unmodified
snapshots of two AOUSD Geometry Working Group proposals, submitted to Pixar's
OpenUSD-proposals repository. They are retained beside the SMLib implementation
so schema, validator, and documentation changes can be reviewed against the
proposal's full design contract. `NOTICE` records their attribution and terms.

- Sources (pull requests opened 2026-05-20):
  - `UsdSolid`: <https://github.com/PixarAnimationStudios/OpenUSD-proposals/pull/109>
  - `cad_geometry`: <https://github.com/PixarAnimationStudios/OpenUSD-proposals/pull/108>
    (also carried, unchanged, by pull request #109)
- Revision: `811f03c603b06d3b175515a4bf991dee8f42f651`, the head of pull request
  #109 (from the AOUSD fork's `p2-usdsolid-schema` branch)
- Upstream subtrees:
  - `proposals/UsdSolid` (`f9b14cbf2d5be6e6f65190a9f074516ef625f7e8`)
  - `proposals/cad_geometry` (`fe220a0f04fe07ec370bfc2247e279f5f195b2a5`)
- Retrieved: 2026-09-16
- Proposal status at that revision: Draft
- Upstream contribution terms: <https://openusd.org/release/contributing_supplemental.html>

The upstream repository does not contain a standalone license or notice file.
This record and `NOTICE` preserve source provenance; they do not replace or
modify the upstream contribution terms.

The included `UsdSolid/schema.usda` is proposal material and identifies itself
as `prelimUsdSolid`. It is not generated, registered, or packaged as a runtime
USD schema plugin. SMLib's runtime schema remains
`source/schema/omniSolid/resources/schema.usda`.

To update the snapshot, replace both complete upstream subtrees from a single
revision and update the revision and tree identifiers above. Do not edit files
inside the snapshots locally.
