# perception

Nothing here yet. This is where YOLO + tracking on real footage will live.

Whatever it does inside, its output has to be one `schemas.messages.Observation`
per camera approach per second, with `source="camera"`. See `docs/schemas.md`
for every field and its unit. Fields the simulator leaves null
(`detector_confidence`, `track_breaks_per_min`, `image_health`) are the ones
perception is expected to fill, because the Guard will use them.

If perception's Observations match the schema, the Guard and controllers run on
them without changes.
