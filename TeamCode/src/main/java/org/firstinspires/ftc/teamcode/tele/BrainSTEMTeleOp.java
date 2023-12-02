package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

@TeleOp (name = "TeleOp", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry);


        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("heading", drive.pose.heading);
            telemetry.addData("Tele Collector State", "TEST");
            telemetry.update();

            if (gamepad1.right_trigger > 0.5) {
                robot.collector.setCollectorIn();
                robot.transfer.setTransferIn();
            } else if (gamepad1.left_trigger > 0.5) {
                robot.collector.setCollectorOut();
                robot.transfer.setTransferOut();
            } else {
               robot.collector.setCollectorOff();
               robot.transfer.setTransferOff();
            }

            if (gamepad1.a) {
                robot.lift.setLiftUp();
              //  robot.depositor.setScoringState();
            } else if (gamepad1.b){
                robot.lift.setLiftDown();
               // robot.depositor.setRestingState();
            } else {
                robot.lift.setLiftOff();
            }

            if (gamepad2.x) {
                robot.hanging.setHangingUnwind();
            } else if (gamepad2.y) {
                robot.hanging.setHangingWind();
            }

            if (gamepad1.left_bumper) {
                robot.hanging.setLockState();
            } else if (gamepad1.right_bumper) {
                robot.hanging.setUnlockState();
                }

            robot.update();

        }
    }
}

