# Selection-invariant axis normalization V1

Status: experimentally validated on 2026-07-20. V0 remains the published and
unchanged production contract.

## Problem

V0 describes every line by an angle and an offset from the centroid of the
whole input selection, sorts by exact angle, and forms groups sequentially.
Consequently, compatible local segments can stop being neighbours when many
unrelated segments are added to the selection.

The required invariant is:

```text
Adding unrelated segments must not prevent an already compatible local axis
from being recognized.
```

## Experimental model

`SelectionInvariantLineNormalizer` is isolated in its own source file and does
not alter `LineNormalizer` V0. It uses:

1. canonical unoriented segment directions;
2. translation-independent pair compatibility;
3. complete-link candidate groups, preventing chain drift;
4. a local origin computed separately for every accepted group;
5. the existing common-axis fit, validation, and interval-union contract.

The research implementation intentionally uses a straightforward quadratic
candidate search. Optimizing the search must not change this mathematical
contract.

## Automated counterexample

The standalone test contains four nearly vertical contiguous segments using
coordinates taken from the real failure, plus 3192 unrelated segments whose
angles lie in the same narrow range.

Expected and observed result:

```text
4 target segments alone             -> 1 group -> 1 segment
4 target + 3192 unrelated segments  -> the same target group -> 1 segment
```

All 25 standalone tests pass. The complete test executable, including the
3196-segment counterexample, ran in approximately 425 ms on the development
machine.

## Real DWG validation

ArchicadDwgOrganizer exposes the experiment through `ADU1` and
`ADO_LINE_UNION_V1`. The existing `ADU` command continues to use V0.

Validated cases:

```text
Selected 4:    groups 1;   replaced 4;    created 1;    reduction 3
Selected 4880: groups 840; replaced 4849; created 1679; reduction 3170
Selected 1102: groups 203; replaced 822;  created 657;  reduction 165
Selected 828:  groups 140; replaced 553;  created 553;  reduction 0
Selected 956:  groups 164; replaced 630;  created 623;  reduction 7
```

In the 4880-object selection, the four segments from the original failure were
confirmed to merge into one segment. A second independent 1102-object DWG
selection also produced the expected result. No groups were rejected in either
large run.

The 828-object case confirms that axis normalization does not bridge real gaps:
553 segments were projected to 140 compatible axes, while the number of
separate intervals remained unchanged. The 956-object case included two axes
whose measured separation was `0.00008453` drawing unit; with an application
tolerance of `0.001`, seven intervals were legitimately collapsed. The
ArchicadDwgOrganizer adapter therefore uses `0.001` as its initial corrective
tolerance, while the CAD-neutral core keeps its conservative default.

This validates the selection-extension invariant for the observed Archicad DWG
cases. It is not yet a claim about every possible geometric arrangement.

## Deliberately excluded next stage

Nominal integer wall-width anchors are not part of this checkpoint. They must
be implemented as a separate detector and validated diagnostically before they
are allowed to influence V1 axes.
