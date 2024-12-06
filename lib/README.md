# Robot Library
[![Build](https://github.com/FRC-4481-Team-Rembrandts/robot-library/actions/workflows/gradle.yml/badge.svg)](https://github.com/FRC-4481-Team-Rembrandts/robot-library/actions/workflows/gradle.yml)
![WPILib](https://img.shields.io/badge/WPILib-2025.1.1--Beta--1-blue)
[![Javadoc](https://img.shields.io/badge/Docs-API_reference-orange)](https://frc-4481-team-rembrandts.github.io/robot-library/)


Library of common season agnostic robot functionality for a *FIRST* Robotics Competition robot.

## Usage
From WPILib 2025.1.1-Beta-1 onwards, the robot library is available as a standalone Gradle project.
The intended way of using the library is to include it as a git submodule in your robot project.
Ensure that there is a reference to the library in your `settings.gradle` file, 
and that the library is included in the `build.gradle` file of the robot.

## Older Versions
By default, the main branch is the most recent version of the library, and contains the newest version of WPILib. 
In case you need an older version of the robot project, please have a look at the releases tab of the repository.

- The oldest supported WPILib version by the standalone robot library is 2025.1.1-Beta-1.
- The oldest supported WPILib version as package in an existing Gradle project is 2024.3.2.

## Contributing
This project is intended to be used across a variety of robots. Therefore, you should carefully consider the following:

- Is this feature useful for (nearly) all robot projects?
- Is this feature easy to understand and use?
- Is this feature something that could be placed in the stock robot instead?

If the feature is indeed suitable for this project, please create a pull request with the changes.

Smaller feature requests can be created as issues, and will be considered for future releases. 
In case of bigger feature requests, please consider creating a fork of this repository.

### Existing Issues
Existing issues may always be worked on. 
If you are working on an issue, please assign yourself to it, so that others know that it is being worked on.
Issues are tagged with labels to indicate their difficulty or area of expertise, as well as a priority.

**Please try to prioritize bugs before enhancements, and high prio before low.**

### Code Formatting
To ensure a consistent code style, we make use of the [Palantir Java Format](https://github.com/palantir/palantir-java-format).
Formatting can automatically be applied by running `./gradlew spotlessApply`.

Formatting will be checked by the CI upon pull request creation. If the formatting is not correct, the CI will fail.