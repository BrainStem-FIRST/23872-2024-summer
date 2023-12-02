package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="Auto Test", group="Robot")
public final class AutoExample extends LinearOpMode {
    @Override

    public enum MarkerLocation {
        FOUR,
        FIVE,
        SIX
    }

    private MarkerLocation markerLocation;

    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(-90)));
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);

//       TODO Camera location function

        markerLocation = MarkerLocation.FOUR;
        double depositTurn = 0;
        double deliveryYPos = 0;

        switch (markerLocation) {
            case FOUR: {
                depositTurn = 90;
                deliveryYPos = -32;
                break;
            }
            case FIVE: {
                depositTurn = 0;
                deliveryYPos = -36;
                break;
            }
            case SIX: {
                depositTurn = -90;
                deliveryYPos = -40;
                break;
            }
        }

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(90))
                                .splineTo(new Vector2d(12, -36), Math.toRadians(-90))
                                .turn(depositTurn)
                                .build(),
                        collector.collectorOutAction(),
                        new SleepAction(1.0),
                        collector.collectorOffAction(),
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(90))
                                .splineTo(new Vector2d(50, deliveryYPos), Math.toRadians(-90))
                                .build()
                )
        );
    }
}
