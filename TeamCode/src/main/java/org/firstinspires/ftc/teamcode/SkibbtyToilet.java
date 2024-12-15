package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class SkibbtyToilet extends LinearOpMode {

    // Declare our motors
    // Make sure your ID's match your configuration
    public DcMotor frontLeftMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backRightMotor = null;


    public DcMotor armMotor = null; //the arm motor
    public CRServo intake = null; //the active intake servo
    public Servo wrist = null; //the wrist servo
    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
To find this, we first need to consider the total gear reduction powering our arm.
First, we have an external 20t:100t (5:1) reduction created by two spur gears.
But we also have an internal gear reduction in our motor.
The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
reduction of ~50.9:1. (more precisely it is 250047/4913:1)
We can multiply these two ratios together to get our final reduction of ~254.47:1.
The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 225 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 205 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 135 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 135 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 195 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = -10 * ARM_TICKS_PER_DEGREE;
    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    @Override

    public void runOpMode() throws InterruptedException {

        double max;
        double raise;


        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        armMotor.setTargetPosition(50);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "frontServo");
        wrist = hardwareMap.get(Servo.class, "turnServo");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


    /* TECH TIP: If Else loops:
    We're using an else if loop on "gamepad1.x" and "gamepad1.b" just in case
    multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
    at the same time. "a" will win over and the intake will turn on. If we just had
    three if statements, then it will set the intake servo's power to multiple speeds in
    one cycle. Which can cause strange behavior. */

            if (gamepad1.a) {
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.x) {
                intake.setPower(INTAKE_OFF);
            } else if (gamepad1.b) {
                intake.setPower(INTAKE_DEPOSIT);
            }


    /* Here we create a "fudge factor" for the arm position.
    This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
    We want the left trigger to move the arm up, and right trigger to move the arm down.
    So we add the right trigger's variable to the inverse of the left trigger. If you pull
    both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
    than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
    The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));



    /* Here we implement a set of if else loops to set our arm to different scoring positions.
    We check to see if a specific button is pressed, and then move the arm (and sometimes
    intake and wrist) to match. For example, if we click the right bumper we want the robot
    to start collecting. So it moves the armPosition to the ARM_COLLECT position,
    it folds out the wrist to make sure it is in the correct orientation to intake, and it
    turns the intake on to the COLLECT mode.*/

            if (gamepad1.right_bumper) {
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                wrist.setPosition(WRIST_FOLDED_IN);
                intake.setPower(INTAKE_COLLECT);
            } else if (gamepad1.left_bumper) {
        /* This is about 20° up from the collecting position to clear the barrier
        Note here that we don't set the wrist position or the intake power when we
        select this "mode", this means that the intake and wrist will continue what
        they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            } else if (gamepad1.y) {
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            } else if (gamepad1.dpad_left) {
        /* This turns off the intake, folds in the wrist, and moves the arm
        back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_right) {
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_up) {
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad1.dpad_down) {
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intake.setPower(INTAKE_OFF);
                wrist.setPosition(WRIST_FOLDED_IN);
            }
            if (gamepad2.a) {
                wrist.setPosition(WRIST_FOLDED_IN);
            } else if (gamepad2.b) {
                wrist.setPosition(WRIST_FOLDED_OUT);
            }

        /* Here we set the target position of our arm to match the variable that was selected
        by the driver.
        We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            // ((DcMotorEx) armMotor).setVelocity(2100);
            ((DcMotorEx) armMotor).setVelocity(500);

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /* TECH TIP: Encoders, integers, and doubles
        Encoders report when the motor has moved a specified angle. They send out pulses which
        only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
        position our arm is currently at can be expressed as a whole number of encoder "ticks".
        The encoder will never report a partial number of ticks. So we can store the position in
        an integer (or int).
        A lot of the variables we use in FTC are doubles. These can capture fractions of whole
        numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
        servo power to 0.5.

        setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
        ticks are always whole numbers, it expects an int. But we want to think about our
        arm position in degrees. And we'd like to be able to set it to fractions of a degree.
        So we make our arm positions Doubles. This allows us to precisely multiply together
        armPosition and our armPositionFudgeFactor. But once we're done multiplying these
        variables. We can decide which exact encoder tick we want our motor to go to. We do
        this by "typecasting" our double, into an int. This takes our fractional double and
        rounds it to the nearest whole number.
        */

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }
            /* send mtelemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
