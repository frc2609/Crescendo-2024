## Controls
### Driver:
| Axis             | Function |
| ---------------- | -------- |
| Left Joystick X  | Drive left/right
| Left Joystick Y  | Drive forward/back
| Right Joystick X | Rotate left/right
| Right Joystick Y | None
| Left Trigger     | Expel note while held
| Right Trigger    | Intake note while held (pause if intake sensor is triggered)

| Button       | Function |
| ------------ | -------- |
| Left Stick   | None
| Right Stick  | None
| Left Bumper  | Auto shoot
| Right Bumper | While held: Raise elevator to amp. When released: Score amp then drop elevator.
| Start        | Reset yaw
| Select       | None
| A            | None
| B            | None
| X            | None
| Y            | None

| DPAD  | Function |
| ----- | -------- |
| Up    | Climber up (while held)
| Down  | Climber down (while held)
| Left  | None
| Right | None

### Operator:
| Button       | Function |
| ------------ | -------- |
| Left Stick   | Add vision estimate from Limelight
| Right Stick  | None
| Left Bumper  | Set shooter to speaker preset
| Right Bumper | Set shooter to podium preset
| Start        | Reset odometry to Limelight
| Back         | Feed note (ignoring shooter status)
| A            | Intake note (stop when intake sensor triggered)
| B            | Expel note
| X            | Turn off intake
| Y            | Feed note when shooter ready

| POV   | Function |
| ----- | -------- |
| Up    | Move elevator to amp
| Down  | Move elevator to intake
| Left  | Set shooter to low throw preset
| Right | Set shooter to high throw preset

## Code Formatting
- Check formatting: `./gradlew spotlessCheck` (output is difficult to read through powershell, so this may be of limited use)
- Apply formatting: `./gradlew spotlessApply`

## Usage Notes
### Alliance Selection in DS
* The AprilTags on the field are for the red alliance.
    * Make sure to specify you are on red alliance in driver station.
* Also, note that the driver station for the red alliance is beside the speaker, so if you sit at the table in the cafeteria, the robot will appear to drive backwards.
    * Grab a chair and set it up beside the speaker if you want to drive the robot properly.

### Autonomous
* Before running an autonomous, you must setup the robot in the starting position (pay attention to the direction it should be pointing too)!
* If you forget to do this the robot will think it is at a different position on the field (potentially hitting obstacles + aiming in the wrong direction) and the gyro will point in a different direction until the Limelight updates the robot odometry.

### Limelight
* If you are testing the robot in the cafeteria, the light from outside significantly affects the Limelight tuning.
    * As it becomes darker, adjust exposure time and black offset from Limelight Configuration.