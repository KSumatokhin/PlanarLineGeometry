# AGENTS.md

## Purpose

PlanarLineGeometry is a CAD-neutral mathematical library. Keep its public model independent of CableCalc, ArchicadDwgOrganizer, BricsCAD, AutoCAD, WPF, DWG objects, layers, colors, and application commands.

## Working rules

- Formulate a small mathematical contract before implementation.
- Stop when special cases begin to replace a clear model.
- Preserve deterministic output independent of input enumeration order.
- Do not round or quantize source geometry.
- Keep CAD extraction, transactions, properties, and output in consumer adapters.
- Add tests before extending normalization or topology.
- Make changes in focused branches and small commits.
- Run the standalone tests and Release build before publishing a meaningful stage.

## Stages

- V0 owns straight-segment common-axis normalization and interval union.
- Polyline decomposition is a separate input adapter/stage.
- T/X detection is a separate topology stage over normalized segments.
- Junction-box placement belongs to the consuming application.
- LINE/POLYLINE assembly is a separate output stage.

Do not expand V0 with topology, CAD, UI, or application-specific branches.
