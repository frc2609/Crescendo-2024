# Swerve Drive Control

## Rotating at Max Speed
* By default, rotating while moving reduces max speed because modules will saturate when rotating and translating the robot.
* It may be more intuitive to find the max speed while rotating and always limit the robot to it, as the robot will not appear to slow down while rotating.

## Cubing
What does it do?
* Open https://www.desmos.com/calculator and type y = x^3, then look between x = 0 and x = 1.
* It makes the joystick input stay low for most of the joystick's range, then quickly ramps up near the end.
* This means the robot is much slower normally, but still reaches its top speed when the driver pushes the joystick to the edge.

### Effects
Makes robot difficult to manouver when one axis is near top speed (e.g. if going full speed forward).
* To try this yourself, move the joystick all the way forward, then try to turn left/right while going that speed. It will be much harder to go slightly left or right, but tilting the joystick past around the halfway point will cause the robot to veer towards the left or right side.
* This occurs because the left/right axis slowly ramps up, so it won't have much effect on the robot while the joystick is between 0-0.5, then suddenly causes the robot to turn once the joystick is pushed enough.
* As a result, the robot handles more like [a snake](https://en.wikipedia.org/wiki/Snake_(video_game_genre)).

To fix this, cube only the forward/backward direction (RELATIVE TO THE FIELD!), because the driver will move this direction more often than left/right.
* Note: You must cube the forward/backward FIELD RELATIVE speed, if you cube the joystick input, the robot will behave completely differently once rotated.

**This may be desireable depending on the driver's preference, as it makes the robot drive as if it were snapping to a grid.**

## Joystick Correction
### Issue
* The robot will drive faster diagonally than straight (left/right or forward/back).

### Why this occurs
Introduction:
* The robot is controlled by a speed and an angle to travel at (as well as an angle for the robot to point at, controlled by a seperate joystick).
* The speed and angle to travel at are calculated using a x speed and a y speed (from the left joystick's x and y axis).
* The angle for the robot to travel at can be calculated using `atan2(y, x)`.
* The speed the robot must travel at can be calculated with the Pythagorean theorem: `sqrt(x^2 + y^2)`.

Issue:
* The controllers we have a maximum value of 1 for the x and y axes.
* This means that while pointing diagonally, the controller would read 1 and 1 (or 1, -1, or -1, 1, etc.) for the x and y axis.
* So, the maximum speed while pointing diagonally is:
```
speed = sqrt(x^2 + y^2)
speed = sqrt(1^2 + 1^2)
speed = sqrt(1 + 1)
speed = sqrt(2)
speed ~= 1.41
```
* Compare this to the speed when the joystick is pointing straight
```
e.g. x = 1, y = 0

speed = sqrt(x^2 + y^2)
speed = sqrt(1^2 + 0^2)
speed = sqrt(1)
speed = 1
```
* Way slower!

### Poor way of fixing this
* Limit the maximum velocity to 1.0 using `Math.min(1.0, speed)`.
* This works when pointing the joystick straight.
* When pointing the joystick diagonally, this will make the robot reach max speed before the joystick hits the edge of the controller, meaning the driver has less control over the robot's speed.
* Example:
```
e.g. x = 1, y = 1

speed = Math.min(1.0, sqrt(1^2 + 1^2))
speed = Math.min(1.0, 1.41)
speed = 1.0

e.g. x = 0.75, y = 0.75

speed = Math.min(1.0, sqrt(0.71^2 + 0.71^2))
speed = Math.min(1.0, 1.0)
speed = 1.0
```
* This isn't good! That means that moving the joystick between 0.71 and 1 will have no effect on the robot's speed.

### Correct way of fixing this
* If you graphed the range of the joystick, you would get a square.
  * Try this for your own controller!
  * Go to https://hardwaretester.com/gamepad, connect a controller, then press "Test Circularity" near the bottom of the screen.
  * For the Xbox controllers we use, the range is close enough to a square to perform the following correction.
* If you create a circle with the radius of the robot's max speed, you'll notice any point on the outside of the circle matches the max speed.
* Therefore, if we map the square to a circle, the robot will travel the same speed when the joystick is pointing straight or diagonally **IF** we map each point on the square to a circle evenly, as different methods of doing this [distort the points on the square differently](https://stackoverflow.com/a/32391780) (the link shows a picture of a circle mapped to a square, but it still illustrates different types of distortion correctly).
* In 2024, we used the method [described here](https://mathproofs.blogspot.com/2005/07/mapping-square-to-circle.html) to map the square joystick readings to a circle.

## Cubing + Joystick Correction
* Cubing first then joystick correction works best.
* Cubing after joystick correction works very poorly (causes the robot to drive way slower diagonally).

### Cube Only
* Max speed mostly limited to 1.0 (so fixes problem joystick correction addresses).
* Issue: At speeds < max speed, diagonal movement is slower than translational movement.

### Cube then correct
* Works the best.

### Correct then cube
* Works very poorly, as diagonal max speed is reduced significantly.

Why does this occur?
* The joystick is not mapped to a square perfectly, so converting it into a circle introduces some errors.
  * e.g. trying to get Driver Station to display the point x = 1, y = 1 with the joystick is impossible
* This error isn't noticeable after the joystick is corrected because it isn't very large (e.g. moving the joystick diagonally towards the bottom left all the way produces 0.95 instead of 1), but cubing the error significantly increases it:
  * 0.95 ^ 3 = 0.86 -> the robot is now driving 14% slower than it should!
* Note that the severity of this effect depends on the controller (and even the direction you point the joystick; they are not very precise), for example, another Xbox controller registered 0.91 pointing towards the top left all the way:
  * 0.91 ^ 3 = 0.75 -> 25% slower than normal!