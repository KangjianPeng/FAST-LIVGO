# Upstream Record

- Project: [FAST-LIVO2-RTK](https://github.com/KangjianPeng/FAST-LIVO2-RTK)
- Imported local branch: `fast-livgo-reproduction`
- Imported commit: `953daa22fbd62d79bb78f7a3acadb9110d450f7a`
- Vendored Sophus commit: `a621ff2e56c56c839a6c40418d42c3c254424b5c`
- Vendored `rpg_vikit_ros2` commit: `fd3f8c9ac04524451a1e49583e0fa9f92d6825d0`

The import also includes three local, uncommitted dataset files from the source
worktree: `config/GREAT_campus01.yaml`,
`config/camera_GREAT_20201027.yaml`, and
`launch/mapping_great_campus01.launch.py`.

The source repository metadata and nested `.git` pointer files were excluded.
Sophus and Vikit are kept as vendored source so this package is tracked as
ordinary files in the ROS 2 branch instead of as nested Git submodules.
