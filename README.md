# aic234471

Current entry and flight source snapshot, arranged as runtime siblings. Keep `entry/` and `flight/` side by side.

## Version provenance

- `entry/` source: 2026-11-02 deployment candidate. SHA-256 of `entry/manifest.json`: `b1072fae48191a96f23df611bb328148481f17db7a9cd71639ca4ae2283401c5`
- `flight/` source: 2026-11-03 deployment candidate. SHA-256 of `flight/MANIFEST_SHA256.json`: `cf6252fdbaddea6977cbc748d3c2c364c2500fc256a4819bb031d3c7491eeec3`
- Both manifest hashes match the deployed snapshot. This establishes source provenance, not that later edits have been deployed or field-validated.

## Layout

- `entry/`: competition entry point, ground-stage flow, configuration, and tests.
- `flight/`: flight flow, vision modules, dependency workspace, and tests.
- Diagnostic logs, flight recordings, maps, deployment backups, historical archives, and local dependency caches are excluded from this Git project and remain in the original workspace.

## Status

This snapshot does not include the canceled and rolled-back dynamic ground-obstacle avoidance prototype. Do not claim that capability is implemented or field-validated. Before deployment or flight testing, review the entry and flight READMEs and manifests, then follow the field preparation and preflight process.

Runtime requirements and external ROS workspace paths are documented in `entry/README_CN.md`, `flight/README_CN.md`, and the scripts. Robot system dependencies and field data are not included.
