^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_nav2_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
