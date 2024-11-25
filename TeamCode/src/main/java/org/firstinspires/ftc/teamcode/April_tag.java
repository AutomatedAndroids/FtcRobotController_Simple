package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class April_tag {
    public class Teleop extends LinearOpMode {

        private Limelight3A limelight;
        Hardware h = new Hardware(hardwareMap);
        private IMU imu;

        @Override
        public void runOpMode() throws InterruptedException {
            limelight = h.limeLight;
            imu = h.imu;


            telemetry.setMsTransmissionInterval(11);

            limelight.pipelineSwitch(0);

            /*
             * Starts polling for data.
             */
            limelight.start();

            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("Botpose", botpose.toString());
                        // print some data for each detected target
                        if (result.isValid()) {
                            // Access fiducial results
                            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                            }
                        }

                        // Access color results
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        for (LLResultTypes.ColorResult cr : colorResults)
                            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

                    telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

                    limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
                    result = limelight.getLatestResult();
                    if (result != null) {
                        if (result.isValid()) {
                            Pose3D botpose = result.getBotpose_MT2();
                        }
                    }
                }
            }
        }
    }
}

