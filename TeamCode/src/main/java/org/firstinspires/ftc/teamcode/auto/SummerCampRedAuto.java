package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.DepositorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.TransferAuto;
@Autonomous(name="Red Auto Backdrop", group="Robot")
public final class SummerCampRedAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(-85)));
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);
        TransferAuto transfer = new TransferAuto(hardwareMap, telemetry);

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(18, 18, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                       // collector.collectorOutAction(),
                       // new SleepAction(0.55),
                       // collector.collectorOffAction()
                        )
                );}}

              /*  // Move robot to backdrop
                drive.updatePoseEstimate();
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .setReversed(true)
                                        .setTangent(Math.toRadians(10))
                                        .splineToLinearHeading(new Pose2d(53,-22, Math.toRadians(-180)), Math.toRadians(0))
                                        .build(),
                                depositor.depositorScoringAction(),
                                new SleepAction(1.0),
                                depositor.pixelDropAction(),
                                new SleepAction(1.0),
                                depositor.depositorRestingAction(),
                                new SleepAction(1.0)
                        )
                );
                drive.updatePoseEstimate();
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading( new Pose2d(48,-65, Math.toRadians(180)), Math.toRadians(180))
                                        .build()
                        )
                );
            }
            if (line == 5) {
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(9, -28, Math.toRadians(90)), Math.toRadians(90))
                                        .build(),
                                collector.collectorOutAction(),
                                new SleepAction(0.55),
                                collector.collectorOffAction()
                        )
                );
                // Move robot to backdrop

                drive.updatePoseEstimate();
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .setReversed(true)
                                        .setTangent(Math.toRadians(-90))
                                        .splineToLinearHeading(new Pose2d(55.5, -30, Math.toRadians(-175)), Math.toRadians(0))
                                        .build(),
                                depositor.depositorScoringAction(),
                                new SleepAction(1.0),
                                depositor.pixelDropAction(),
                                new SleepAction(1.0),
                                depositor.depositorRestingAction(),
                                new SleepAction(1.0)
                        )
                );


                drive.updatePoseEstimate();
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading( new Pose2d(48,-65, Math.toRadians(180)), Math.toRadians(180))
                                        .build()
                        )
                );
            }
            if (line == 6) {
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .setTangent(Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(15.5, -24, Math.toRadians(0)), Math.toRadians(90))
                                        .build(),
                                collector.collectorOutAction(),
                                new SleepAction(0.55),
                                collector.collectorOffAction()
                                // Move robot to backdrop
                        )
                );

                drive.updatePoseEstimate();
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .setReversed(true)
//                                    .setTangent(Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(14,-50), Math.toRadians(-90))
                                        .build()
                        )
                );
                drive.updatePoseEstimate();
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .setReversed(true)
                                        .setTangent(Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(56.5, -38, Math.toRadians(180)), Math.toRadians(0))
                                        .build(),
                                depositor.depositorScoringAction(),
                                new SleepAction(1.0),
                                depositor.pixelDropAction(),
                                new SleepAction(1.0),
                                depositor.depositorRestingAction(),
                                new SleepAction(1.0)
                        )
                );

                drive.updatePoseEstimate();
                Actions.runBlocking(
                        new SequentialAction(
                                drive.actionBuilder(drive.pose)
                                        .splineToLinearHeading( new Pose2d(50,-65, Math.toRadians(180)), Math.toRadians(180))
                                        .build()
                        )
                );

            }
        }
    }




}*/
