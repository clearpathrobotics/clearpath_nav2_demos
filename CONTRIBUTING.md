# Contributing to clearpath_nav2_demos

Thanks for your interest in improving `clearpath_nav2_demos`! This repository provides example
[Nav2](https://docs.nav2.org/) and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
configurations and launch files for mapping, localization, and autonomous navigation on Clearpath
platforms. It is a **downstream demo** package that assumes a robot (real or simulated) is already
running. Please read the notes below before opening a pull request.

## Getting started

1. Fork the repository and clone your fork.
2. Create a feature branch off `jazzy`:

   ```bash
   git checkout -b my-feature jazzy
   ```

3. Build the workspace and source it:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

4. Install the pre-commit hooks (one-time setup):

   ```bash
   pip install pre-commit
   pre-commit install
   ```

## Linting

This repository uses [pre-commit](https://pre-commit.com/) to run linting and formatting checks
(trailing whitespace, end-of-file, YAML/JSON checks, `markdownlint`, and `flake8`) before each
commit. Run them against the whole tree before pushing:

```bash
pre-commit run --all-files
```

## Where things live

See the [Layout section of the README](README.md#layout) for the full map:

- [`launch/`](launch) — the `slam`, `localization`, and `nav2` launch files.
- [`config/`](config) — per-platform parameter files (`a200`, `a300`, `dd100`, `j100`, `w200`, …).
- [`maps/`](maps) — sample maps matching the simulator worlds.

When adding support for a platform, add a matching directory under `config/` rather than
special-casing an existing one.

## Testing your changes

These demos do **not** start a robot, so test against a running robot or the simulator:

```bash
# In one terminal, start a simulated robot
ros2 launch clearpath_gz simulation.launch.py

# In another, run the demo you changed
ros2 launch clearpath_nav2_demos slam.launch.py
```

Confirm the demo launches cleanly and behaves correctly for the platform whose config you touched.
Mismatched footprints or sensor frames are a common cause of poor navigation behavior, so verify
the config matches the platform.

## Continuous integration

[`clearpath_nav2_demos_ci`](.github/workflows/ci.yml) runs on every pull request:

- **jazzy** (`build_and_test`) — builds and tests against the released `testing`/`main` repos.
- **Jazzy Clearpath Source** (`source_build`) — source build of `clearpath_nav2_demos`.

Both jobs build against **released** dependencies and do not pull in upstream source branches, so
they are not affected by in-progress branches in other Clearpath repositories — they should pass on
their own. If a job fails, the cause is in this repository (or an already-released upstream
dependency), not an unmerged upstream branch.

## Submitting a pull request

1. Make sure the workspace builds and the affected demo launches correctly.
2. Push your branch and open a pull request against `jazzy`.
3. Write a clear description of what the change does and the platform/world you tested against.

## Reporting issues

Please open issues on the
[GitHub issue tracker](https://github.com/clearpathrobotics/clearpath_nav2_demos/issues) and fill
out the bug report template, which walks you through the details we need to reproduce the problem.

## License

By contributing, you agree that your contributions will be licensed under the
[BSD-3-Clause license](LICENSE) that covers this project.
