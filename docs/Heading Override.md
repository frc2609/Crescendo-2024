# Swerve Drive Heading Override
* `Drive::overrideHeading()` can be used to have the robot align to a particular heading while driving.
* PathPlanner offers this functionality as well, however we do not use it because our implementation functions in teleop as well (so the driver can retain control during e.g. AprilTagTrackDrive).