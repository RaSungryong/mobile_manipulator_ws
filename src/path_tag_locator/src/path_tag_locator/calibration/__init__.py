"""
path_tag_locator.calibration
============================
Map calibration workflow:

- :mod:`plan_io`       — loads ``reference_tags.yaml`` (multi-ref world
                          poses) and ``calibration_plan.yaml`` (ordered
                          path_tag -> ref_tag assignments).
- :mod:`map_io`        — reads ``map.yaml``, atomically writes
                          ``map_updated.yaml`` preserving non-tag
                          structure.
- :mod:`orchestrator`  — :class:`CalibrationOrchestrator` runs the full
                          session, driving base + arm and persisting
                          per-tag results.
"""
