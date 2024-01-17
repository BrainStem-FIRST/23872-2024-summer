package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotTele.PresentationRobotTele;
import org.firstinspires.ftc.teamcode.robotTele.DepositorTele;

@TeleOp (name = "PresenTeleOp", group = "Robot")
public class PresentationTeleOp extends LinearOpMode {
    private boolean retractionInProgress = false;
    private ElapsedTime waitForHolder = new ElapsedTime();
    private final ElapsedTime retractionTime = new ElapsedTime();
    private final ElapsedTime restingTime = new ElapsedTime();


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        PresentationRobotTele robot = new PresentationRobotTele(hardwareMap, telemetry);
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

//collector and transfer
            if (gamepad2.right_trigger > 0.2) {
                robot.collector.setCollectorIn();
                robot.collector.setCollectorState();
                robot.transfer.setTransferIn();
                robot.transfer.transferState();
            } else if (gamepad2.left_trigger > 0.2) {
                robot.collector.setCollectorOut();
                robot.collector.setCollectorState();
                robot.transfer.setTransferOut();
                robot.transfer.transferState();
            } else if (!(gamepad2.right_trigger > 0.2) && !(gamepad2.left_trigger > 0.2)) {
                robot.collector.setCollectorOff();
                robot.collector.setCollectorState();
                robot.transfer.setTransferOff();
                robot.transfer.transferState();
            }

//hanging wind
                if (gamepad2.y) {
                    robot.hanging.setHangingUnwind();
                } else if (gamepad2.x) {
                    robot.hanging.setHangingWind();
                } else {
                    robot.hanging.setHangingRest();
                }

//drone release
                if (gamepad2.left_bumper) {
                    robot.drone.setClaspServo();
                } else if (gamepad2.right_bumper) {
                    robot.drone.setReleaseServo();
                }

//lift and depositor

                increaseLiftButton.update(gamepad2.a);
                decreaseLiftButton.update(gamepad2.b);
                if (increaseLiftButton.getState()) {
                    robot.depositor.setScoringState();
                    robot.lift.increaseLevel();
                    robot.lift.updateLevel();
                } else if (decreaseLiftButton.getState()) {
                    robot.lift.decreaseLevel();
                    robot.lift.updateLevel();
                }

                robot.update();
            }
        }
    }





