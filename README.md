## Controls

## Code Formatting
- Check formatting: `./gradlew spotlessCheck` (output is difficult to read through powershell, so this may be of limited use)
- Apply formatting: `./gradlew spotlessApply`

## Usage Notes
### Autonomous
* Before running an autonomous, you must:
    * Actual robot: Setup the robot in the starting position (pay attention to the direction it should be pointing too)!
    * Simulation: Zero the gyro (robot will teleport to starting position as long as the gyro is facing 0).
* If you forget to do this:
    * Actual robot: Robot will think it is at a different position on the field (potentially hitting obstacles + aiming in the wrong direction) and the gyro will point in a different direction until the Limelight updates the robot odometry.
    * Simulation: Auto will run (teleports to position), but the robot's heading will be offset.