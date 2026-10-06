"""
path_tag_locator.calibration
============================
Map calibration workflow:

- :mod:`plan_io`       — loads ``reference_tags.yaml`` (multi-ref world
                          poses) and a plan file
                          (``calibration_plan_plate{1,2}.yaml``, ordered
                          path_tag -> ref_tag assignments).
- :mod:`map_io`        — reads ``map.yaml`` (read-only metadata) and
                          atomically writes ``map_world_<ts>.yaml``; it
                          never writes map.yaml.
- :mod:`orchestrator`  — :class:`CalibrationOrchestrator` runs the full
                          session, driving base + arm and persisting
                          per-tag results.
"""
