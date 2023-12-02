package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotTele.BrainSTEMRobotTele;
import org.firstinspires.ftc.teamcode.robotTele.DepositorTele;

@TeleOp (name = "TeleOp", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    private boolean retractionInProgress = false;
    private final ElapsedTime retractionTime = new ElapsedTime();


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        BrainSTEMRobotTele robot = new BrainSTEMRobotTele(hardwareMap, telemetry);
        double power = 0.0;
        StickyButton stickyButtonRightBumper = new StickyButton();
        StickyButton stickyButtonLeftBumper = new StickyButton();
        StickyButton stickyButtonA = new StickyButton();
        StickyButton stickyButtonB = new StickyButton();
        ToggleButton toggleButtonRightTrigger = new ToggleButton();
        ToggleButton toggleButtonLeftTrigger = new ToggleButton();
        ElapsedTime elapsedTime = new ElapsedTime();


        waitForStart();

        while (opModeIsActive()) {

            if (robot.depositor.depositorServoState == DepositorTele.DepositorServoState.SCORING) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.4,
                                -gamepad1.left_stick_x * 0.4
                        ),
                        -gamepad1.right_stick_x * 0.4));


            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x * 0.75));
            }

            drive.updatePoseEstimate();

            if (gamepad1.left_stick_y == 1.0){
                robot.collector.setCollectorOff();
                robot.depositor.pixelHold();
            }

            telemetry.addData("Tele Collector State", "TEST");
            telemetry.update();
//collector
            toggleButtonLeftTrigger.update(gamepad1.left_trigger > 0.2);
            toggleButtonRightTrigger.update(gamepad1.right_trigger > 0.2);
            if (toggleButtonRightTrigger.getState()) {
                robot.collector.setCollectorIn();
                robot.collector.setCollectorState();
                robot.transfer.setTransferIn();
                robot.transfer.transferState();
            } else if (toggleButtonLeftTrigger.getState()) {
                robot.collector.setCollectorOut();
                robot.collector.setCollectorState();
                robot.transfer.setTransferOut();
                robot.transfer.transferState();
            } else {
                robot.collector.setCollectorOff();
                robot.collector.setCollectorState();
                robot.transfer.setTransferOff();
                robot.transfer.transferState();
            }

//pixel holder
            if (gamepad1.right_bumper) {
                retractionTime.reset();
                retractionInProgress = true;
                robot.depositor.setDropState();
            } else if (gamepad1.left_bumper) {
                robot.depositor.setHoldState();
                toggleButtonRightTrigger.update(false);
                toggleButtonLeftTrigger.update(false);
            }

            if (retractionInProgress){
                if (retractionTime.seconds() > 0.5){
                    robot.depositor.setRestingState();
                }
                if (retractionTime.seconds() > 1.0){
                    robot.lift.levelCounter = 0;
                    retractionInProgress = false;
                    robot.collector.setCollectorOff();
                }
            }
//depositor
            if (gamepad1.x) {
                robot.depositor.setRestingState();
            } else if (gamepad1.y) {
                robot.depositor.setScoringState();
            }
//hanging wind
            if (gamepad2.x) {
                robot.hanging.setHangingUnwind();
            } else if (gamepad2.y) {
                robot.hanging.setHangingWind();
            }
//hanging servo
            if (gamepad2.a) {
                robot.hanging.setLockState();
            } else if (gamepad2.b) {
                robot.hanging.setUnlockState();
                telemetry.addLine("hanging unlock");

            }
                }
//drone release
//            if (gamepad2.a) {
//                robot.drone.setClaspServo();
//            } else if (gamepad2.b) {
//                robot.drone.setReleaseServo();
//            }
//lift and depositor

            stickyButtonA.update(gamepad1.a);
            stickyButtonB.update(gamepad1.b);
            if (stickyButtonA.getState()) {
                robot.depositor.setScoringState();
                    robot.lift.increaseLevel();
                    robot.lift.updateLevel();
            } else if (stickyButtonB.getState()) {
                robot.depositor.setRestingState();
                robot.lift.decreaseLevel();
                robot.lift.updateLevel();
            }

            stickyButtonLeftBumper.update(gamepad2.left_bumper);
            stickyButtonRightBumper.update(gamepad2.right_bumper);
            if (stickyButtonLeftBumper.getState()) {
                robot.depositor.setScoringState();
                robot.lift.increaseLevel();
                robot.lift.updateLevel();
            } else if (stickyButtonRightBumper.getState()) {
                robot.depositor.setRestingState();
                robot.lift.decreaseLevel();
                robot.lift.updateLevel();
            }


//            stickyButtonA.update(gamepad1.a);
//            if (stickyButtonA.getState()) {
//                robot.lift.setLiftZero();
//                telemetry.addLine("LiftZero");
//            }

            robot.update();
            telemetry.addData("State of right trigger",toggleButtonRightTrigger.getState());
        }
    }

