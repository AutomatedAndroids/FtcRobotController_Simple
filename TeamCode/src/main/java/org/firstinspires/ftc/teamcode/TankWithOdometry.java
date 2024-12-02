/* MIT License
 * Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
*/

@TeleOp(name="goBILDA® PinPoint Odometry Example", group="Linear OpMode")
//@Disabled

public class TankWithOdometry extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    SparkFunOTOS myOtos;

    double oldTime = 0;


    @Override
    public void runOpMode() {

// Initialize the hardware variables. Note that the strings used here must correspond
// to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

/*
Set the odometry pod positions relative to the point that the odometry computer tracks around.
The X pod offset refers to how far sideways from the tracking point the
X (forward) odometry pod is. Left of the center is a positive number,
right of center is a negative number. the Y pod offset refers to how far forwards from
the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
backwards is a negative number.
*/
//odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(0, 0); //these are tuned for 3110-0002-0001 Product Insight #1

/*
Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
number of ticks per mm of your odometry pod.
*/
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//odo.setEncoderResolution(13.26291192);


/*
Set the direction that each of the two odometry pods count. The X (forward) pod should
increase when you move the robot forward. And the Y (strafe) pod should increase when
you move the robot to the left.
*/
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


/*
Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
The IMU will automatically calibrate when first powered on, but recalibrating before running
the robot is a good idea to ensure that the calibration is "good".
resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
This is recommended before you run your autonomous, as a bad initial calibration can cause
an incorrect starting value for x, y, and heading.
*/
//odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

//OTOS
// Get a reference to the sensor
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

// All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

// Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();


// run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//OTOS
// Get the latest position, which includes the x and y coordinates, plus the
// heading angle
            SparkFunOTOS.Pose2D pos = myOtos.getPosition();

// Reset the tracking if the user requests it
            if (gamepad1.y) {
                myOtos.resetTracking();
            }

// Re-calibrate the IMU if the user requests it
            if (gamepad1.x) {
                myOtos.calibrateImu();
            }


// Update the telemetry on the driver station
            telemetry.update();
// Inform user of available controls
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

// Log the position to the telemetry
// BX - swap X and Y for sparkfun OTOs and change the unit to mm
            telemetry.addData("Otos X coordinate mm", 25.4*pos.y);
            telemetry.addData("Otos Y coordinate mm", 25.4*pos.x);
            telemetry.addData("Otos Heading angle", pos.h);

/*
Request an update from the Pinpoint odometry computer. This checks almost all outputs
from the device in a single I2C read.
*/
            odo.update();

/*
Optionally, you can update only the heading of the device. This takes less time to read, but will not
pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
*/
//odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);


            if (gamepad1.a){
                odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.b){
                odo.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

/*
This code prints the loop frequency of the REV Control Hub. This frequency is effected
by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
of time each cycle takes and finds the frequency (number of updates per second) from
that cycle time.
*/
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


/*
gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
*/
            Pose2D pos2 = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos2.getX(DistanceUnit.MM), pos2.getY(DistanceUnit.MM), pos2.getHeading(AngleUnit.DEGREES));
            telemetry.addData("ODO - Position", data);

/*
gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
*/
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("ODO - Velocity", velocity);


/*
Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
READY: the device is working as normal
CALIBRATING: the device is calibrating and outputs are put on hold
NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
*/
            telemetry.addData("ODO - Status", odo.getDeviceStatus());

            telemetry.addData("ODO - Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("ODO - REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();

        }
    }
    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

// Set the desired units for linear and angular measurements. Can be either
// meters or inches for linear, and radians or degrees for angular. If not
// set, the default is inches and degrees. Note that this setting is not
// persisted in the sensor, so you need to set at the start of all your
// OpModes if using the non-default value.
// myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
// myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

// Assuming you've mounted your sensor to a robot and it's not centered,
// you can specify the offset for the sensor relative to the center of the
// robot. The units default to inches and degrees, but if you want to use
// different units, specify them before setting the offset! Note that as of
// firmware version 1.0, these values will be lost after a power cycle, so
// you will need to set them each time you power up the sensor. For example, if
// the sensor is mounted 5 inches to the left (negative X) and 10 inches
// forward (positive Y) of the center of the robot, and mounted 90 degrees
// clockwise (negative rotation) from the robot's orientation, the offset
// would be {-5, 10, -90}. These can be any value, even the angle can be
// tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

// Here we can set the linear and angular scalars, which can compensate for
// scaling issues with the sensor measurements. Note that as of firmware
// version 1.0, these values will be lost after a power cycle, so you will
// need to set them each time you power up the sensor. They can be any value
// from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
// first set both scalars to 1.0, then calibrate the angular scalar, then
// the linear scalar. To calibrate the angular scalar, spin the robot by
// multiple rotations (eg. 10) to get a precise error, then set the scalar
// to the inverse of the error. Remember that the angle wraps from -180 to
// 180 degrees, so for example, if after 10 rotations counterclockwise
// (positive rotation), the sensor reports -15 degrees, the required scalar
// would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
// robot a known distance and measure the error; do this multiple times at
// multiple speeds to get an average, then set the linear scalar to the
// inverse of the error. For example, if you move the robot 100 inches and
// the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

// The IMU on the OTOS includes a gyroscope and accelerometer, which could
// have an offset. Note that as of firmware version 1.0, the calibration
// will be lost after a power cycle; the OTOS performs a quick calibration
// when it powers up, but it is recommended to perform a more thorough
// calibration at the start of all your OpModes. Note that the sensor must
// be completely stationary and flat during calibration! When calling
// calibrateImu(), you can specify the number of samples to take and whether
// to wait until the calibration is complete. If no parameters are provided,
// it will take 255 samples and wait until done; each sample takes about
// 2.4ms, so about 612ms total
        myOtos.calibrateImu();

// Reset the tracking algorithm - this resets the position to the origin,
// but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

// After resetting the tracking, the OTOS will report that the robot is at
// the origin. If your robot does not start at the origin, or you have
// another source of location information (eg. vision odometry), you can set
// the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

// Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

}
