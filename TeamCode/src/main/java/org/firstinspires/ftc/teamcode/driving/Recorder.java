package org.firstinspires.ftc.teamcode.driving;

import android.os.Environment;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

@Autonomous(name="PLAYBACK", group="B")
class Playback extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftDrive = null; //the left drivetrain motor
    public DcMotor rightDrive = null; //the right drivetrain motor
    public DcMotor armMotor = null; //the arm motor
    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo
    public DcMotor leftActuator = null;
    public DcMotor rightActuator = null;


    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 260 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 235 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 165 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 165 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 125 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 20 * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to
    intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to
    when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    String logFilePath = String.format("%s/FIRST/data/mylog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    File file = new File(logFilePath);
    BufferedReader reader;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double raise;

        try {
            reader = new BufferedReader(new FileReader(file));

            double time = Double.parseDouble(reader.readLine());
            gamepad1.a  = Boolean.parseBoolean(reader.readLine());
            gamepad1.a  = Boolean.parseBoolean(reader.readLine());
            gamepad1.x  = Boolean.parseBoolean(reader.readLine());
            gamepad1.y  = Boolean.parseBoolean(reader.readLine());

            gamepad1.left_bumper    = Boolean.parseBoolean(reader.readLine());
            gamepad1.right_bumper   = Boolean.parseBoolean(reader.readLine());

            gamepad1.left_trigger  = Float.parseFloat(reader.readLine());
            gamepad1.right_trigger  = Float.parseFloat(reader.readLine());

            gamepad1.left_stick_x  = Float.parseFloat(reader.readLine());
            gamepad1.left_stick_y  = Float.parseFloat(reader.readLine());
            gamepad1.right_stick_x  = Float.parseFloat(reader.readLine());
            gamepad1.right_stick_y  = Float.parseFloat(reader.readLine());

            gamepad1.dpad_up    = Boolean.parseBoolean(reader.readLine());
            gamepad1.dpad_left  = Boolean.parseBoolean(reader.readLine());
            gamepad1.dpad_right = Boolean.parseBoolean(reader.readLine());
            gamepad1.dpad_down  = Boolean.parseBoolean(reader.readLine());

            gamepad1.left_stick_button  = Boolean.parseBoolean(reader.readLine());
            gamepad1.right_stick_button = Boolean.parseBoolean(reader.readLine());

            gamepad1.start  = Boolean.parseBoolean(reader.readLine());
            gamepad1.back   = Boolean.parseBoolean(reader.readLine());

            gamepad2.a  = Boolean.parseBoolean(reader.readLine());
            gamepad2.a  = Boolean.parseBoolean(reader.readLine());
            gamepad2.a  = Boolean.parseBoolean(reader.readLine());
            gamepad2.a  = Boolean.parseBoolean(reader.readLine());
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        /* Define and Initialize Motors */
        leftDrive = hardwareMap.get(DcMotor.class, "leftWheel"); //the left
        rightDrive = hardwareMap.get(DcMotor.class, "rightWheel"); //the right
        armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor

        leftActuator = hardwareMap.get(DcMotor.class, "leftAct");
        rightActuator = hardwareMap.get(DcMotor.class, "rightAct");

        /* Most skid-steer/differential drive robots require reversing one
motor to drive forward.
for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


/* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This
causes the motor to slow down
much faster when it is coasting. This creates a much more controllable
drivetrain. As the robot
stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/*This sets the maximum current that the control hub will apply to the
arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


/* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to
stop and reset encoder.
If you do not have the encoder plugged into this motor, it will not
run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftActuator.setDirection(DcMotor.Direction.FORWARD);
        rightActuator.setDirection(DcMotor.Direction.FORWARD);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "frontServo");
        wrist = hardwareMap.get(Servo.class, "turnServo");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* define and init Limelight3A sensors */
        telemetry.setMsTransmissionInterval(11);

        /* Limelights start polling data. */

        /* Send telemetry message to signify limelight status */
        telemetry.addLine("Limelight started.");

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();



        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            try {
                reader = new BufferedReader(new FileReader(file));

                double time = Double.parseDouble(reader.readLine());
                gamepad1.a  = Boolean.parseBoolean(reader.readLine());
                gamepad1.a  = Boolean.parseBoolean(reader.readLine());
                gamepad1.x  = Boolean.parseBoolean(reader.readLine());
                gamepad1.y  = Boolean.parseBoolean(reader.readLine());

                gamepad1.left_bumper    = Boolean.parseBoolean(reader.readLine());
                gamepad1.right_bumper   = Boolean.parseBoolean(reader.readLine());

                gamepad1.left_trigger  = Float.parseFloat(reader.readLine());
                gamepad1.right_trigger  = Float.parseFloat(reader.readLine());

                gamepad1.left_stick_x  = Float.parseFloat(reader.readLine());
                gamepad1.left_stick_y  = Float.parseFloat(reader.readLine());
                gamepad1.right_stick_x  = Float.parseFloat(reader.readLine());
                gamepad1.right_stick_y  = Float.parseFloat(reader.readLine());

                gamepad1.dpad_up    = Boolean.parseBoolean(reader.readLine());
                gamepad1.dpad_left  = Boolean.parseBoolean(reader.readLine());
                gamepad1.dpad_right = Boolean.parseBoolean(reader.readLine());
                gamepad1.dpad_down  = Boolean.parseBoolean(reader.readLine());

                gamepad1.left_stick_button  = Boolean.parseBoolean(reader.readLine());
                gamepad1.right_stick_button = Boolean.parseBoolean(reader.readLine());

                gamepad1.start  = Boolean.parseBoolean(reader.readLine());
                gamepad1.back   = Boolean.parseBoolean(reader.readLine());

                gamepad2.a  = Boolean.parseBoolean(reader.readLine());
                gamepad2.a  = Boolean.parseBoolean(reader.readLine());
                gamepad2.a  = Boolean.parseBoolean(reader.readLine());
                gamepad2.a  = Boolean.parseBoolean(reader.readLine());
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
            timer.reset();


//            telemetry.update();

/* Set the drive and turn variables to follow the joysticks on the gamepad.
the joysticks decrease as you push them up. So reverse the Y axis. */
            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;


/* Here we "mix" the input channels together to find the power to
apply to each motor.
The both motors need to be set to a mix of how much you're retesting
the robot move
forward, and how much you're requesting the robot turn. When you ask
the robot to rotate
the right and left motors need to move in opposite directions. So we
will add rotate to
forward for the left motor, and subtract rotate from forward for the
right motor. */

            left = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            /* Set the motor power to the variables we've mixed and normalized */
            leftDrive.setPower(left);
            rightDrive.setPower(right);



/* Here we handle the three buttons that have direct control of the
intake speed.
These control the continuous rotation servo that pulls elements into the robot,
If the user presses A, it sets the intake power to the final variable that
holds the speed we want to collect at.
If the user presses X, it sets the servo to Off.
And if the user presses B it reveres the servo to spit out the element.*/

/* TECH TIP: If Else loops:
We're using an else if loop on "gamepad1.x" and "gamepad1.b" just in case
multiple buttons are pressed at the same time. If the driver presses
both "a" and "x"
at the same time. "a" will win over and the intake will turn on. If we just had
three if statements, then it will set the intake servo's power to
multiple speeds in
one cycle. Which can cause strange behavior. */

            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }


/* Here we create a "fudge factor" for the arm position.
This allows you to adjust (or "fudge") the arm position slightly with
the gamepad triggers.
We want the left trigger to move the arm up, and right trigger to move
the arm down.
So we add the right trigger's variable to the inverse of the left
trigger. If you pull
both triggers an equal amount, they cancel and leave the arm at zero.
But if one is larger
than the other, it "wins out". This variable is then multiplied by our
FUDGE_FACTOR.
The FUDGE_FACTOR is the number of degrees that we can adjust the arm
by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger +
                    (-gamepad1.left_trigger));



/* Here we implement a set of if else loops to set our arm to
different scoring positions.
We check to see if a specific button is pressed, and then move the arm
(and sometimes
intake and wrist) to match. For example, if we click the right bumper
we want the robot
to start collecting. So it moves the armPosition to the ARM_COLLECT position,
it folds out the wrist to make sure it is in the correct orientation
to intake, and it
turns the intake on to the COLLECT mode.*/

            if(gamepad2.x) {
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            else if(gamepad2.y) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            if(gamepad1.right_bumper){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            }

            else if (gamepad1.left_bumper){
/* This is about 20° up from the collecting position to clear the barrier
Note here that we don't set the wrist position or the intake power when we
select this "mode", this means that the intake and wrist will continue what
they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
/* This turns off the intake, folds in the wrist, and moves the arm
back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK+7;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

/* Here we set the target position of our arm to match the variable
that was selected
by the driver.
We also set the target velocity (speed) the motor runs at, and use
setMode to run it.*/
            armMotor.setTargetPosition((int) (armPosition +armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

/* TECH TIP: Encoders, integers, and doubles
Encoders report when the motor has moved a specified angle. They send
out pulses which
only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This
means that the
position our arm is currently at can be expressed as a whole number of
encoder "ticks".
The encoder will never report a partial number of ticks. So we can
store the position in
an integer (or int).
A lot of the variables we use in FTC are doubles. These can capture
fractions of whole
numbers. Which is great when we want our arm to move to 122.5°, or we
want to set our
servo power to 0.5.

setTargetPosition is expecting a number of encoder ticks to drive to.
Since encoder
ticks are always whole numbers, it expects an int. But we want to
think about our
arm position in degrees. And we'd like to be able to set it to
fractions of a degree.
So we make our arm positions Doubles. This allows us to precisely
multiply together
armPosition and our armPositionFudgeFactor. But once we're done
multiplying these
variables. We can decide which exact encoder tick we want our motor to
go to. We do
this by "typecasting" our double, into an int. This takes our
fractional double and
rounds it to the nearest whole number.
*/

/* Check to see if our arm is over the current limit, and report via
telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* setup Actuator position */
            raise = -gamepad2.right_stick_y;
            leftActuator.setPower(raise);
            rightActuator.setPower(raise);


            telemetry.addData("left actuator", leftActuator.getCurrentPosition());
            telemetry.addData("right actuator", rightActuator.getCurrentPosition());
            telemetry.addData("wrist position", wrist.getPosition());

            telemetry.update();

/* send telemetry to the driver of the arm's current position and
target position */
//telemetry.addData("armTarget: ", armMotor.getTargetPosition());
//telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
//telemetry.update();
            if (timer.milliseconds() > time) {
                if (timer.milliseconds()-time > 40) { // this is the number of dealy we can have before we skip over lines to the next segment.
                    try {
                        for (int i = 0; i < 24; i++) {
                            reader.readLine();
                        }
                    } catch (IOException e) {
                        throw new RuntimeException(e);
                    }
                }
            } else {
                Thread.sleep((long) (time-timer.milliseconds()));
            }
        }
    }
}
@TeleOp(name = "RECORDER", group="B")
public class Recorder extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftDrive = null; //the left drivetrain motor
    public DcMotor rightDrive = null; //the right drivetrain motor
    public DcMotor armMotor = null; //the arm motor
    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo
    public DcMotor leftActuator = null;
    public DcMotor rightActuator = null;


    final double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 260 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 235 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 165 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 165 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 125 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 20 * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to
    intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to
    when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    String logFilePath = String.format("%s/FIRST/data/mylog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
    File file = new File(logFilePath);
    FileWriter writer;

    ElapsedTime timer = new ElapsedTime();

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double raise;
        try {
            writer = new FileWriter(file);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }


        /* Define and Initialize Motors */
        leftDrive = hardwareMap.get(DcMotor.class, "leftWheel"); //the left
        rightDrive = hardwareMap.get(DcMotor.class, "rightWheel"); //the right
        armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor

        leftActuator = hardwareMap.get(DcMotor.class, "leftAct");
        rightActuator = hardwareMap.get(DcMotor.class, "rightAct");

        /* Most skid-steer/differential drive robots require reversing one
motor to drive forward.
for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


/* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This
causes the motor to slow down
much faster when it is coasting. This creates a much more controllable
drivetrain. As the robot
stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/*This sets the maximum current that the control hub will apply to the
arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


/* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to
stop and reset encoder.
If you do not have the encoder plugged into this motor, it will not
run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftActuator.setDirection(DcMotor.Direction.FORWARD);
        rightActuator.setDirection(DcMotor.Direction.FORWARD);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "frontServo");
        wrist = hardwareMap.get(Servo.class, "turnServo");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);


        /* Send telemetry message to signify limelight status */
        telemetry.addLine("Limelight started.");

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();



        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            /* display Limelight data. */
            timer.reset();
            try {
                writer.write("" + timer.milliseconds());
                writer.write("\n" + gamepad1.a);
                writer.write("\n" + gamepad1.b);
                writer.write("\n" + gamepad1.x);
                writer.write("\n" + gamepad1.y);
                writer.write("\n" + gamepad1.left_bumper);
                writer.write("\n" + gamepad1.right_bumper);
                writer.write("\n" + gamepad1.left_trigger);
                writer.write("\n" + gamepad1.right_trigger);
                writer.write("\n" + gamepad1.left_stick_x);
                writer.write("\n" + gamepad1.left_stick_y);
                writer.write("\n" + gamepad1.right_stick_x);
                writer.write("\n" + gamepad1.right_stick_y);
                writer.write("\n" + gamepad1.dpad_up);
                writer.write("\n" + gamepad1.dpad_left);
                writer.write("\n" + gamepad1.dpad_down);
                writer.write("\n" + gamepad1.dpad_right);
                writer.write("\n" + gamepad1.left_stick_button);
                writer.write("\n" + gamepad1.right_stick_button);
                writer.write("\n" + gamepad1.start);
                writer.write("\n" + gamepad1.back);
                writer.write("\n" + gamepad2.a);
                writer.write("\n" + gamepad2.b);
                writer.write("\n" + gamepad2.x);
                writer.write("\n" + gamepad2.y);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
            timer.reset();



//            telemetry.update();

/* Set the drive and turn variables to follow the joysticks on the gamepad.
the joysticks decrease as you push them up. So reverse the Y axis. */
            forward = gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;


/* Here we "mix" the input channels together to find the power to
apply to each motor.
The both motors need to be set to a mix of how much you're retesting
the robot move
forward, and how much you're requesting the robot turn. When you ask
the robot to rotate
the right and left motors need to move in opposite directions. So we
will add rotate to
forward for the left motor, and subtract rotate from forward for the
right motor. */

            left = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            /* Set the motor power to the variables we've mixed and normalized */
            leftDrive.setPower(left);
            rightDrive.setPower(right);



/* Here we handle the three buttons that have direct control of the
intake speed.
These control the continuous rotation servo that pulls elements into the robot,
If the user presses A, it sets the intake power to the final variable that
holds the speed we want to collect at.
If the user presses X, it sets the servo to Off.
And if the user presses B it reveres the servo to spit out the element.*/

/* TECH TIP: If Else loops:
We're using an else if loop on "gamepad1.x" and "gamepad1.b" just in case
multiple buttons are pressed at the same time. If the driver presses
both "a" and "x"
at the same time. "a" will win over and the intake will turn on. If we just had
three if statements, then it will set the intake servo's power to
multiple speeds in
one cycle. Which can cause strange behavior. */

            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            }
            else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }


/* Here we create a "fudge factor" for the arm position.
This allows you to adjust (or "fudge") the arm position slightly with
the gamepad triggers.
We want the left trigger to move the arm up, and right trigger to move
the arm down.
So we add the right trigger's variable to the inverse of the left
trigger. If you pull
both triggers an equal amount, they cancel and leave the arm at zero.
But if one is larger
than the other, it "wins out". This variable is then multiplied by our
FUDGE_FACTOR.
The FUDGE_FACTOR is the number of degrees that we can adjust the arm
by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger +
                    (-gamepad1.left_trigger));



/* Here we implement a set of if else loops to set our arm to
different scoring positions.
We check to see if a specific button is pressed, and then move the arm
(and sometimes
intake and wrist) to match. For example, if we click the right bumper
we want the robot
to start collecting. So it moves the armPosition to the ARM_COLLECT position,
it folds out the wrist to make sure it is in the correct orientation
to intake, and it
turns the intake on to the COLLECT mode.*/

            if(gamepad2.x) {
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            else if(gamepad2.y) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            if(gamepad1.right_bumper){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_OUT);
                intake.setPower(INTAKE_COLLECT);
            }

            else if (gamepad1.left_bumper){
/* This is about 20° up from the collecting position to clear the barrier
Note here that we don't set the wrist position or the intake power when we
select this "mode", this means that the intake and wrist will continue what
they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.y){
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
/* This turns off the intake, folds in the wrist, and moves the arm
back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK+7;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }

/* Here we set the target position of our arm to match the variable
that was selected
by the driver.
We also set the target velocity (speed) the motor runs at, and use
setMode to run it.*/
            armMotor.setTargetPosition((int) (armPosition +armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

/* TECH TIP: Encoders, integers, and doubles
Encoders report when the motor has moved a specified angle. They send
out pulses which
only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This
means that the
position our arm is currently at can be expressed as a whole number of
encoder "ticks".
The encoder will never report a partial number of ticks. So we can
store the position in
an integer (or int).
A lot of the variables we use in FTC are doubles. These can capture
fractions of whole
numbers. Which is great when we want our arm to move to 122.5°, or we
want to set our
servo power to 0.5.

setTargetPosition is expecting a number of encoder ticks to drive to.
Since encoder
ticks are always whole numbers, it expects an int. But we want to
think about our
arm position in degrees. And we'd like to be able to set it to
fractions of a degree.
So we make our arm positions Doubles. This allows us to precisely
multiply together
armPosition and our armPositionFudgeFactor. But once we're done
multiplying these
variables. We can decide which exact encoder tick we want our motor to
go to. We do
this by "typecasting" our double, into an int. This takes our
fractional double and
rounds it to the nearest whole number.
*/

/* Check to see if our arm is over the current limit, and report via
telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* setup Actuator position */
            raise = -gamepad2.right_stick_y;
            leftActuator.setPower(raise);
            rightActuator.setPower(raise);


            telemetry.addData("left actuator", leftActuator.getCurrentPosition());
            telemetry.addData("right actuator", rightActuator.getCurrentPosition());
            telemetry.addData("wrist position", wrist.getPosition());

            telemetry.update();

/* send telemetry to the driver of the arm's current position and
target position */
//telemetry.addData("armTarget: ", armMotor.getTargetPosition());
//telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
//telemetry.update();


        }
    }
}
