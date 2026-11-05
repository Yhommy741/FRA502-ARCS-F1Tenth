-- Cartographer 2D localization (pure localization) on a saved map (.pbstream).
-- Same settings as mapping; only the last few submaps are kept, so the saved map
-- is used and not extended.
--
-- The launch file starts the localization trajectory WITH the known start pose
-- (start_trajectory service, initial pose = map origin = where mapping started).
-- So Cartographer only has to track the car near where it is: no whole-map
-- search, which on long similar-looking straights (Austin) snapped the pose to a
-- wrong spot ~9 m further along the track.

include "cartographer_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
-- Matches are applied (and the "connected" timer below reset) at every optimization:
-- every 10 scans = ~1 s while driving
POSE_GRAPH.optimize_every_n_nodes = 10

-- How the car is corrected onto the saved map (Cartographer pose_graph_2d.cc):
--   the start pose connects the localization trajectory to the saved map; after that
--   only LOCAL matches (scan vs saved submaps near the current estimate) correct it,
--   and every match renews the connection. If no match is found for
--   global_constraint_search_after_n_seconds, Cartographer switches to whole-map
--   (global) search. With global search switched off (ratio 0), one quiet stretch
--   meant the car was never corrected again: it just built its own new map ("mapping
--   instead of localizing"), drifted and hit a wall. Now a guarded global search
--   recovers it.
--
-- Accept local matches more easily (0.75 rejected most of them -> no corrections)
POSE_GRAPH.constraint_builder.min_score = 0.6
-- Try a local match for more node/submap pairs (default 0.3)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
-- Recovery: after this many seconds without any local match the car is treated as
-- lost and only the whole-map (global) search below runs, until it matches again.
-- 4 s: well above the ~1 s between optimizations (shorter = normal gaps count as lost
-- and switch off the local search), but the car is not driven 20-30 m lost (10 s default).
POSE_GRAPH.global_constraint_search_after_n_seconds = 4.
-- Whole-map search for a lost car: rare (2 % of scans) and only very good matches
-- (0.9), so look-alike spots (~9 m further along an Austin straight) are rejected.
POSE_GRAPH.global_sampling_ratio = 0.02
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.9
-- Match only near the current estimate (defaults 15 m / 7 m allow look-alike spots)
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 2.

return options
