package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

/** @noinspection unused*/
public class Hardware {

    // Consts
    public Double lateralEncoderDist = 12.96;
    public Double longEncoderDist = 238.125;

    // Motors
    public static DcMotor leftWheel, rightWheel, arm, slider1, slider2;
    public static CRServo intake;
    public static Servo wrist;
    public static IMU imu;


    public static Limelight3A limeLight;

    HardwareMap hardwareMap;


    public Hardware(HardwareMap map) {
        resetHardwareMap(map);

    }
    /** for those times when u update the hardware map in the middle of running and require that *fresh* information. */
    public void resetHardwareMap(HardwareMap map) {
        this.hardwareMap = map;

        hardwareMap.get(DcMotor.class, "topLeft");

        leftWheel = this.hardwareMap.get(DcMotor.class, "leftWheel");
        rightWheel = this.hardwareMap.get(DcMotor.class, "rightWheel");
        arm = this.hardwareMap.get(DcMotor.class, "arm");

        intake = this.hardwareMap.get(CRServo.class, "frontServo");
        wrist = this.hardwareMap.get(Servo.class, "turnServo");

        slider1 = this.hardwareMap.get(DcMotor.class, "leftAct");
        slider2 = this.hardwareMap.get(DcMotor.class, "rightAct");

        imu = this.hardwareMap.get(IMU.class, "imu");


        leftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limeLight = this.hardwareMap.get(Limelight3A.class, "limelight");
    }
}