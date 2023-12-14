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
    private ElapsedTime waitForHolder = new ElapsedTime();
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
        StickyButton gamepad2StickyButtonA = new StickyButton();
        StickyButton gamepad2StickyButtonB = new StickyButton();
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

            if (gamepad2.left_stick_button) {
                robot.lift.setRawPower(-.25);
            }
            if (gamepad2.right_stick_button) {
                robot.lift.resetEncoders();

            }

            if (gamepad1.left_stick_y == 1.0) {
                robot.collector.setCollectorOff();
                robot.depositor.pixelHold();
            }

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

//depositor
            if (gamepad1.x) {
                robot.depositor.setRestingState();
            } else if (gamepad1.y) {
                robot.depositor.setScoringState();
            }
            if (retractionInProgress) {
                if (retractionTime.seconds() > 1.0) {
                    robot.depositor.setRestingState();
                }
                if (retractionTime.seconds() > 1.0) {
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

            gamepad2StickyButtonA.update(gamepad2.a);
            gamepad2StickyButtonB.update(gamepad2.b);
            if (gamepad2StickyButtonA.getState()) {
                robot.depositor.setScoringState();
                robot.lift.increaseLevel();
                robot.lift.updateLevel();
            } else if (gamepad2StickyButtonB.getState()) {
                robot.lift.decreaseLevel();
                robot.lift.updateLevel();
            }


            robot.update();
        }
    }
}



