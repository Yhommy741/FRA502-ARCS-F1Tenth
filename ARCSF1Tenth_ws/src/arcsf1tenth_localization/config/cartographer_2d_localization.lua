-- Cartographer 2D localization (pure localization) on a saved map (.pbstream).
-- Same settings as mapping; only the last few submaps are kept, so the saved map
-- is used and not extended.

include "cartographer_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

return options
