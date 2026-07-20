# PlanarLineGeometry

`PlanarLineGeometry.Core` is a small CAD-neutral library for deterministic normalization of nearly collinear two-dimensional line segments.

The validated V0 operation:

```text
LINE segments
-> common-axis normalization
-> projection to one-dimensional intervals
-> interval union
-> normalized LINE segments and diagnostics
```

The library has no dependency on CableCalc, ArchicadDwgOrganizer, BricsCAD, AutoCAD, WPF, or DWG types. CAD applications explicitly select and read entities, call the library, and write the result in their own transaction.

## Validated V0

V0 was checked in BricsCAD on a real DWG exported from Archicad:

- 831 selected lines;
- 554 source lines in 112 accepted groups;
- 358 normalized result lines;
- 277 unchanged lines;
- 196-line net reduction;
- no rejected groups or invalid lines.

The first consumers are CableCalc and ArchicadDwgOrganizer. Smart Offset is a planned consumer of the later composed pipeline.

## Experimental V1

The selection-invariant axis experiment is recorded in
[`SELECTION_INVARIANCE_V1.md`](SELECTION_INVARIANCE_V1.md). It is isolated from
the validated V0 and has been checked on the original 3196-segment
counterexample and two real DWG selections.

## Build and test

```powershell
dotnet build PlanarLineGeometry.sln -c Release
.\PlanarLineGeometry.Tests\bin\Release\net472\PlanarLineGeometry.Tests.exe
```

Current target: .NET Framework 4.7.2 (`net472`).

The conservative defaults are `0.00001` drawing unit for linear comparison and `0.1°` for angular comparison. Applications may pass a larger explicit tolerance when corrective geometry movement is intended. Partial-overlap union and adjacent-interval union are independent settings; exact duplicates are always collapsed.

## License

MIT. See [`LICENSE`](LICENSE).

## Scope

V0 processes straight segments only. Polyline decomposition, T/X topology, junction-box placement, and line-to-polyline assembly are separate future stages and must not be added as branches inside the validated interval-union algorithm.
