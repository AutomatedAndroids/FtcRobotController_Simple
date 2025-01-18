/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="POV: score auto but ur teammate is a bummmmmmmm", group="Robot")
public class TeammateIsABumAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftDrive, rightDrive, armMotor, leftActuator, rightActuator;
    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo
    private ElapsedTime     runtime = new ElapsedTime();
    final double ARM_TICKS_PER_DEGREE = 19.7924893140647;
    final double ARM_ATTACH_HANGING_HOOK = 133   * ARM_TICKS_PER_DEGREE;
    static final double     FORWARD_SPEED = 0.5;
    static final double     ARM_FORWARD_SPEED = 0.8;
    static final double     TURN_SPEED    = 0.5;
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;
    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive = hardwareMap.get(DcMotor.class, "leftWheel"); //the left
        rightDrive = hardwareMap.get(DcMotor.class, "rightWheel"); //the right
        armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor

        leftActuator = hardwareMap.get(DcMotor.class, "leftAct");
        rightActuator = hardwareMap.get(DcMotor.class, "rightAct");

        intake = hardwareMap.get(CRServo.class, "frontServo");
        wrist = hardwareMap.get(Servo.class, "turnServo");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftActuator.setDirection(DcMotor.Direction.FORWARD);
        rightActuator.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();




        leftDrive.setPower(FORWARD_SPEED);
        rightDrive.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.4)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        //Put the arm in scoring settings
        armMotor.setTargetPosition((int) (ARM_ATTACH_HANGING_HOOK));

        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set the wrist position
        wrist.setPosition(WRIST_FOLDED_IN);


        sleep(1000);
        leftActuator.setPower(ARM_FORWARD_SPEED);
        rightActuator.setPower(ARM_FORWARD_SPEED);
        sleep(1900);


        leftActuator.setPower(0);
        rightActuator.setPower(0);

        armMotor.setTargetPosition((int) (ARM_ATTACH_HANGING_HOOK));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2500);

        armMotor.setTargetPosition((int) (ARM_ATTACH_HANGING_HOOK));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(WRIST_FOLDED_OUT);
        leftDrive.setPower(0.3);
        rightDrive.setPower(0.3);


        sleep(500);
        leftDrive.setPower(0.5);
        rightDrive.setPower(-0.5);
        sleep(900);
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.3);
        sleep(900);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        wrist.setPosition(WRIST_FOLDED_OUT);
        armMotor.setTargetPosition((int) (ARM_ATTACH_HANGING_HOOK));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(0.7);


        sleep(2600);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        wrist.setPosition(WRIST_FOLDED_IN);
        armMotor.setTargetPosition((int) (0));
        ((DcMotorEx) armMotor).setVelocity(2100);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(400);
        intake.setPower(0);
        leftActuator.setPower(-ARM_FORWARD_SPEED);
        rightActuator.setPower(-ARM_FORWARD_SPEED);


        sleep(1900);

        leftDrive.setPower(-0.3);
        rightDrive.setPower(-0.3);
        sleep(1500);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
