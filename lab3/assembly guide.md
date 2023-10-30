# Robot assembly guide

## Tools required
- #1 Phillips screwdriver
- Pliers or 6mm wrench
- Double sided tape or hot glue

## List of parts

This is an overview of all the parts included with the kit:

<img src="/images/overview.jpg" width="50%"/>

- 1 pack metal chassis kit
  - 2 chassis plates
  - 2 motor mounting brackets
- 1 pack wheel kit
  - 2 plastic wheel rims
  - 2 silicone rubber tires
  - 2 self-tapping screws
- 2 TT plastic motors
- 1 pack ball roller
  - 1 ball roller
  - 2 mounting screws
  - 2 nuts
- 1 long L bracket
- 1 battery charger
- 1 li-po battery
- 1 pack brass standoffs
  - 4 25mm (1 inch) brass standoffs
  - 8 3mm M3 screws
- 3 6mm male-female standoffs
- 4 5mm M3 screws
- 4 M3 nuts
- 1 pack motor mounting screws
  - 4 long M3 screws
  - 4 M3 nuts
 
### Other parts
- Arduino Mega 2560
- HC-SR04 ultrasonic distance sensor
- male-female jumper wires
- L293D motor driver IC
- Mega prototyping breadboard shield

### Optional parts
- 1 pack slotted encoder wheel pair
- 2 T-slot photo interrupters

## Assembling the wheels

Stretch the tires onto the plastic wheel rims carefully.

<img src="/images/wheels_assembled.jpg" width="50%"/>

## Assembling the motors

The parts for one motor assembly are shown below:

<img src="/images/motor_parts.jpg" width="50%"/>

The motor mounting screws fit into two holes on the sides of the motors. They attach the motors to the mounting bracket. Take care when tightening the screws since the plastic motor case can crack when overtightened.

### Optional: wheel encoders
The T-slot photo interrupters and slotted encoder wheels are optional. They allow the robot to sense the wheel rotation speed, but they are tricky to mount and don't fit perfectly in the chassis. Highly recommended to assemble the kit without them first and only add them if needed.

The completed motor assembly should look something like the picture below:

<img src="/images/motors_left_right.jpg" width="50%"/>

Note that the nuts that secure the motor have to have their flats on the top and bottom, since they hit the chassis if they are diagonal.

<img src="/images/motor_assembled.jpg" width="50%"/>


## Chassis assembly

Next, mount three standoffs onto one of the chassis plates as shown below:

<img src="/images/chassis_standoffs.jpg" width="50%"/>

This will be the bottom plate. Make sure to have the plate at the right orientation since it is easy to have it flipped the wrong way around.

Place the motors onto the bottom plate, making sure that the slots on the mounting bracket line up with their corresponding holes:

<img src="/images/motors_on_chassis.jpg" width="50%"/>

The motors will be loosely attached at this point, so be careful not to have them pop out of place. The top plate we prepare next will hold it more securely.

Prepare the top chassis plate by attaching standoffs to the Arduino Mega. We specifically want only these three mounting holes which correspond to slots on the chassis plate:

<img src="/images/arduino_standoffs.jpg" width="50%"/>

Mount the arduino onto the chassis plate and secure it with nuts on the underside. We want it to look something like this:

<img src="/images/arduino_on_chassis.jpg" width="50%"/>

Once the standoffs are secured to the chassis plate, we need to remove the Arduino in order to gain access to the mounting holes underdeath it. Leave the standoffs on the chassis plate and unscrew the Arduino from the top. Once it's off, we can place the top chassis plate onto the bottom chassis + motors assembly and secure everything into place.

When attaching the top plate, make sure to have the motor mounting brackets slot into their corresponding holes:

<img src="/images/chassis_assembled.jpg" width="50%"/>

Use the remaining short M3 screws to tightly secure the top plate. Don't reattach the Arduino yet, since we still need space to mount other attachments.

Mount the metal roller ball to the front of the robot using the included screws. The plastic is very soft so be careful not to overtighten the screws. There is a chance that the screws might pull straight through the plastic.

<img src="/images/ball_roller.jpg" width="50%"/>

## Attaching the wheels

You can attach the wheels at this point. Make sure to secure them by adding a screw at the center, or else the wheels can come off when the robot makes a turn.

## Battery and charger

Flip the robot upside-down to mount the battery and its charging PCB. Ideally you should use double-sided tape to secure it since the heat from hot glue can damage the battery. Take note of the orientation of the charging PCB so the plug isn't blocked.

<img src="/images/battery_attached.jpg" width="50%"/>

## Distance sensor

Flip the robot over again to attach the HC-SR04 distance sensor. It mounts using the long L bracket and a single screw and nut. The sensor can be attached with either hot glue or double sided tape, but ideally it should be oriented vertically like in the picture to maximize its performance.

<img src="/images/ultrasonic.jpg" width="50%"/>

## Final touches

Reattach the Arduino and do some cable management to avoid loose wires from getting caught in the wheels. The robot is now ready for use.

Note that the motors should not be connected directly to the Arduino since they draw too much current which can damage it. You can make a motor driver using parts from the ELEGOO kit and attach it to the robot in order to make the motors run.
