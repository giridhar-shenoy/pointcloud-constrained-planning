# Third-party components

Files in this repository that were copied or derived from other projects.

| Component | Source | License | Changes |
|---|---|---|---|
| `.clang-format` | [ament_lint / ament_clang_format `configuration/.clang-format`](https://github.com/ament/ament_lint/blob/rolling/ament_clang_format/ament_clang_format/configuration/.clang-format) | Apache-2.0 | Copied verbatim (ROS 2 C++ style). |
| `src/pcp_direct/include/pcp_direct/traj/trapezoidal.hpp` | My own repository [ur5e-teaching-controllers](https://github.com/giridhar-shenoy/ur5e-teaching-controllers), file [`src/ur5e_teaching_controllers/src/profiles/trapezoidal.hpp`](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/blob/52fc1ba/src/ur5e_teaching_controllers/src/profiles/trapezoidal.hpp) at commit [52fc1ba](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/commit/52fc1ba) on its `main` branch (same author) | MIT | Namespace changed to `pcp_direct::profiles`; origin comment added; the header's pointer to `docs/controllers.md` now names that repository, where the file lives. Listed here only because it is a copy from another repository. |

ur5e-teaching-controllers is private until it is published; until then the links above return
404, and they resolve unchanged once it is public. The commit exists on its `main` branch.
