^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_nav2_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.9.0 (2026-10-29)
------------------
* Nav2 updates (`#55 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/55>`_)
  * Updated Nav2 behavior and platform configs.
  * Added SLAM localization and posegraph modes.
  * Added office map and update warehouse map.
  * Fixed pre-commit.
  * Updated README for SLAM modes and office map.
* Bump https://github.com/PyCQA/flake8 from 7.3.0 to 7.4.1
  Bumps [https://github.com/PyCQA/flake8](https://github.com/PyCQA/flake8) from 7.3.0 to 7.4.1.
  - [Commits](https://github.com/PyCQA/flake8/compare/7.3.0...7.4.1)
  ---
  updated-dependencies:
  - dependency-name: https://github.com/PyCQA/flake8
  dependency-version: 7.4.1
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
* Bump https://github.com/igorshubovych/markdownlint-cli from v0.44.0 to 0.49.1 (`#48 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/48>`_)
  * Bump https://github.com/igorshubovych/markdownlint-cli
  Bumps [https://github.com/igorshubovych/markdownlint-cli](https://github.com/igorshubovych/markdownlint-cli) from v0.44.0 to 0.49.1.
  - [Release notes](https://github.com/igorshubovych/markdownlint-cli/releases)
  - [Commits](https://github.com/igorshubovych/markdownlint-cli/compare/v0.44.0...v0.49.1)
  ---
  updated-dependencies:
  - dependency-name: https://github.com/igorshubovych/markdownlint-cli
  dependency-version: 0.49.1
  dependency-type: direct:production
  ...
  * Fixed linting for this PR.
  ---------
  Co-authored-by: dependabot[bot] <49699333+dependabot[bot]@users.noreply.github.com>
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
* Bump https://github.com/pre-commit/pre-commit-hooks from v5.0.0 to 6.0.0
  Bumps [https://github.com/pre-commit/pre-commit-hooks](https://github.com/pre-commit/pre-commit-hooks) from v5.0.0 to 6.0.0.
  - [Release notes](https://github.com/pre-commit/pre-commit-hooks/releases)
  - [Changelog](https://github.com/pre-commit/pre-commit-hooks/blob/main/CHANGELOG.md)
  - [Commits](https://github.com/pre-commit/pre-commit-hooks/compare/v5.0.0...v6.0.0)
  ---
  updated-dependencies:
  - dependency-name: https://github.com/pre-commit/pre-commit-hooks
  dependency-version: 6.0.0
  dependency-type: direct:production
  ...
* Bump actions/github-script from 7 to 9
  Bumps [actions/github-script](https://github.com/actions/github-script) from 7 to 9.
  - [Release notes](https://github.com/actions/github-script/releases)
  - [Commits](https://github.com/actions/github-script/compare/v7...v9)
  ---
  updated-dependencies:
  - dependency-name: actions/github-script
  dependency-version: '9'
  dependency-type: direct:production
  update-type: version-update:semver-major
  ...
* Bump https://github.com/PyCQA/flake8 from 7.1.2 to 7.3.0
  Bumps [https://github.com/PyCQA/flake8](https://github.com/PyCQA/flake8) from 7.1.2 to 7.3.0.
  - [Commits](https://github.com/PyCQA/flake8/compare/7.1.2...7.3.0)
  ---
  updated-dependencies:
  - dependency-name: https://github.com/PyCQA/flake8
  dependency-version: 7.3.0
  dependency-type: direct:production
  update-type: version-update:semver-minor
  ...
* Added Github PR templates, precommit and contributing notes.  Also, u… (`#44 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/44>`_)
  * Added Github PR templates, precommit and contributing notes.  Also, updated CI, updated issue templates and README.
  * Added dependabot.
  * Fixed invalid mergify param.
* Contributors: Tony Baltovski, dependabot[bot]

2.8.1 (2026-07-16)
------------------
* Tuned slam_toolbox defaults: increase throttling, map update interval, and minimum travel thresholds. (`#42 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/42>`_)
* Fixed global costmap rolling window and collision monitor tuning. (`#40 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/40>`_)
* Remaped /tf and /tf_static into robot namespace in nav2.launch.py. (`#38 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/38>`_)
* Changed to rostooling/setup-ros-docker:ubuntu-noble-latest for CI image. (`#36 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/36>`_)
* Added mergify and auto-label. (`#34 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/34>`_)
* Contributors: Tony Baltovski

2.8.0 (2025-12-12)
------------------
* Add `scan_topic` launch argument (`#32 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/32>`_)
* Contributors: Chris Iverach-Brereton

2.7.1 (2025-09-10)
------------------
* Fix SLAM parameter rewrites for Jazzy (`#31 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/31>`_)
  * Set the scan_topic parameter to the fully qualified topic, set the map_name parameter
  * Add a note about the scan_topic and map_name parameters getting rewritten by the launch file
* Contributors: Chris Iverach-Brereton

2.7.0 (2025-08-25)
------------------
* Accept setup path with or without a trailing slash (`#30 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/30>`_)
* Contributors: Hilary Luo

2.5.0 (2025-05-30)
------------------
* Fix typos in issue templates (`#27 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/27>`_)
* Add parameter files for Dingo (all variants), Ridgeback (`#25 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/25>`_)
  * Add parameter files for Dingo (all variants), Ridgeback
  * Add trailing newlines to config files, add missing remap to launch file
* Fix the slam launch file to use the new lifecycle node (`#24 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/24>`_)
* Contributors: Chris Iverach-Brereton, Hilary Luo

2.4.0 (2025-04-30)
------------------
* Update nav2 configuration files for Jazzy (`#21 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/21>`_)
* use_sim_time should not be in the yaml in Jazzy (`#19 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/19>`_)
* Contributors: Chris Iverach-Brereton, Hilary Luo

2.0.0 (2025-01-31)
------------------
* Enable stamped cmd_vel messages for Nav2
* Add Nav2 config files for A300. Not yet tested on the physical robot (`#18 <https://github.com/clearpathrobotics/clearpath_nav2_demos/issues/18>`_)
* Remove repos file from CI; it doesn't exist in this repo
* Add source CI
* Update CI for Jazzy
* Fix import ordering, allow shadowing of builtin
* Contributors: Chris Iverach-Brereton

0.2.0 (2024-01-22)
------------------
* Increased inflation radius
* Added W200 configs
* Contributors: Roni Kreinin

0.1.0 (2023-09-15)
------------------
* Updated get model function and remapped scan topics
  * Update get model function to match clearpath config
  * Remap to correct namespacing on scan topics
* Contributors: Hilary Luo

0.0.2 (2023-07-27)
------------------
* Updated sensor namespace
* Linter fix
* Contributors: Roni Kreinin

0.0.1 (2023-07-17)
------------------
* Added space to package.xml.
* Updated package.xml order.
* Added test dependencies.
* Added Github actions.
* Config updates
* Namespacing fixes
* Updated odom topics
* Fixed footprint and adjusted velocity/accel limits on J100
* Use config to set namespace and platform model
* Current best parameters for Husky
* Cleanup
* Jackal params
* Initial commit
* Initial commit
* Contributors: Hilary Luo, Roni Kreinin, Tony Baltovski
