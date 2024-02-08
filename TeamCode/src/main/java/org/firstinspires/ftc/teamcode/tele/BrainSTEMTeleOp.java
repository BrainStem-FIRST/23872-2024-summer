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
import org.firstinspires.ftc.teamcode.robotTele.LiftTele;

@TeleOp (name = "TeleOp", group = "Robot")
public class BrainSTEMTeleOp extends LinearOpMode {
    private boolean retractionInProgress = false;
    private ElapsedTime waitForHolder = new ElapsedTime();
    private final ElapsedTime retractionTime = new ElapsedTime();
    private final ElapsedTime restingTime = new ElapsedTime();
    private int rightBumperCounter = 0;


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        BrainSTEMRobotTele robot = new BrainSTEMRobotTele(hardwareMap, telemetry);
        double power = 0.0;
        StickyButton stickyButtonRightBumper = new StickyButton();
        StickyButton stickyButtonLeftBumper = new StickyButton();
        ToggleButton toggleButtonRightTrigger = new ToggleButton();
        ToggleButton toggleButtonLeftTrigger = new ToggleButton();
        StickyButton increaseLiftButton = new StickyButton();
        StickyButton decreaseLiftButton = new StickyButton();
        StickyButton stickyButtonX = new StickyButton();
        StickyButton stickyButtonY = new StickyButton();
        ElapsedTime elapsedTime = new ElapsedTime();


        waitForStart();

        while (opModeIsActive()) {

            if (robot.depositor.depositorServoState == DepositorTele.DepositorServoState.SCORING) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * 0.45,
                                -gamepad1.left_stick_x * 0.45
                        ),
                        -gamepad1.right_stick_x * 0.45));


            } else {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x * 0.75));
            }

            drive.updatePoseEstimate();

            telemetry.addData("Tele Collector State", "TEST");
            telemetry.update();
//collector
            if (gamepad1.right_trigger > 0.2) {
                robot.collector.setCollectorIn();
                robot.collector.setCollectorState();
                robot.transfer.setTransferIn();
                robot.transfer.transferState();
            } else if (gamepad1.left_trigger > 0.2) {
                robot.collector.setCollectorOut();
                robot.collector.setCollectorState();
                robot.transfer.setTransferOut();
                robot.transfer.transferState();
            } else if (!(gamepad1.right_trigger > 0.2) && !(gamepad1.left_trigger > 0.2)) {
                robot.collector.setCollectorOff();
                robot.collector.setCollectorState();
                robot.transfer.setTransferOff();
                robot.transfer.transferState();
                if (gamepad2.a&& gamepad2.b){
                    robot.collector.DrawbridgeFive();

                }
            }

//pixel holder
            stickyButtonRightBumper.update(gamepad1.right_bumper);
            stickyButtonLeftBumper.update(gamepad1.left_bumper);
            if (stickyButtonRightBumper.getState()) {
                rightBumperCounter += 1;
            }
            if (stickyButtonLeftBumper.getState()) {
                toggleButtonRightTrigger.update(false);
                toggleButtonLeftTrigger.update(false);
                robot.depositor.stateCounter = 2;
                robot.depositor.updateState();
            }

            if (rightBumperCounter == 1) {
                if (stickyButtonRightBumper.getState()) {
                    robot.depositor.decreaseState();
                    robot.depositor.updateState();
                }
            }
            if (rightBumperCounter == 2) {
                rightBumperCounter = 0;
                robot.depositor.decreaseState();
                robot.depositor.updateState();
                if (gamepad1.right_bumper) {
                    retractionTime.reset();
                    retractionInProgress = true;
                }
            }

//depositor
            stickyButtonX.update(gamepad1.x);
            stickyButtonY.update(gamepad1.y);
            if (stickyButtonX.getState()) {
                robot.depositor.setRestingState();
            } else if (stickyButtonY.getState()) {
                robot.depositor.setScoringState();
            }
            if (retractionInProgress) {
//                if (retractionTime.seconds() > 0.05) {
//                    robot.lift.increaseLevel();
//                    robot.lift.updateLevel();
////                }
                    if (retractionTime.seconds() > 0.5) {
                        robot.depositor.setRestingState();
                    }
                    if (retractionTime.seconds() > 0.75) {
                        robot.lift.setLiftZero();
                        robot.lift.levelCounter = 0;
                        retractionInProgress = false;
                        robot.collector.setCollectorOff();
                    }
                }
//hanging wind
                if (gamepad2.y) {
                    robot.hanging.setHangingUnwind();
                } else if (gamepad2.x) {
                    robot.hanging.setHangingWind();
                } else {
                    robot.hanging.setHangingRest();
                }
////hanging servo
                if (gamepad2.left_trigger > 0.2) {
                    robot.hanging.setLockState();
                } else if (gamepad2.right_trigger > 0.2) {
                    robot.hanging.setUnlockState();
                    telemetry.addLine("hanging unlock");
                }


//drone release
                if (gamepad2.left_bumper) {
                    robot.drone.setClaspServo();
                } else if (gamepad2.right_bumper) {
                    robot.drone.setReleaseServo();
                }
//lift and depositor


//            if (gamepad2.right_stick_button && gamepad2.left_stick_button) {
//                robot.lift.resetEncoders();
//                robot.lift.updateLevel();
//
//                            }

//            raiseDepositorButton.update(gamepad1.a);
//            lowerDepositorButton.update(gamepad1.b);
//            if (raiseDepositorButton.getState()) {
//                robot.depositor.setScoringState();
//                robot.lift.increaseLevel();
//                robot.lift.updateLevel();
//            } else if (lowerDepositorButton.getState()) {
//                robot.depositor.setRestingState();
//                robot.lift.decreaseLevel();
//                robot.lift.updateLevel();
//            }

                increaseLiftButton.update(gamepad1.a);
                decreaseLiftButton.update(gamepad1.b);
                if (increaseLiftButton.getState()) {
                    robot.depositor.setScoringState();
                    robot.lift.increaseLevel();
                    robot.lift.updateLevel();
                } else if (decreaseLiftButton.getState()) {
                    robot.lift.decreaseLevel();
                    robot.lift.updateLevel();
                }


                robot.update();
                telemetry.addData("rightBumperCounter", rightBumperCounter);
                telemetry.addData("stateCounter", robot.depositor.stateCounter);
                telemetry.addData("left servo pos", robot.depositor.LeftDepositor.getPosition());
                telemetry.addData("right servo pos", robot.depositor.RightDepositor.getPosition());
                telemetry.update();
            }
        }
    }





