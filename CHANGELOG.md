# Change log

<!-- ## [Unreleased]
### Fixed
- Fixed definition/structure files where `std_msgs::Float64` and `std_msgs::Int32` were using pointers in the struct instead of actual value.


## [0.2.0] - 2020-04-17 -->

## [1.0.1] - 2020-04-21
#### Added
- This changelog.
#### Changed
- Have all topics and TF frame to be prefixed with `podi` to prevent collision in multi-robot situations.
- Changed the differential driver gazebo plugin to use the `podi` prefix
- Changed some arguments in roslaunch file to use lower_underscore_case instead of Camelcase.

## [1.0.0] - 2020-XX-XX
Initial Release with face, msgs, description, and control components.
