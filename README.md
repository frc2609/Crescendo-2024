## Controls

## Code Formatting
- Check formatting: `./gradlew spotlessCheck` (output is difficult to read through powershell, so this may be of limited use)
- Apply formatting: `./gradlew spotlessApply`

## Usage Notes
### Autonomous
* Before running an autonomous, you must setup the robot in the starting position (pay attention to the direction it should be pointing too)!
* If you forget to do this the robot will think it is at a different position on the field (potentially hitting obstacles + aiming in the wrong direction) and the gyro will point in a different direction until the Limelight updates the robot odometry.

### Limelight
* If you are testing the robot in the cafeteria, the light from outside significantly affects the Limelight tuning.
    * As it becomes darker, adjust exposure time and black offset from Limelight Configuration.