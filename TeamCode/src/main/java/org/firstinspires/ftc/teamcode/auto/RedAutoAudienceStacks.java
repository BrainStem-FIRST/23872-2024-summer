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

@Autonomous(name="Red Auto Audience Stacks", group="Robot")
public final class RedAutoAudienceStacks extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private HuskyLens.Block[] blocks;

    private Pose2d startPose = new Pose2d(-35.25, -62.50, Math.toRadians(-90));

    // Determine the prop position
    int targetTagPos = -1;
    int targetBlockPos = -1; // The block of interest within the blocks array.

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        //Set starting position
        drive.pose = startPose;
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);
        TransferAuto transfer = new TransferAuto(hardwareMap, telemetry);


        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();


        blocks = huskyLens.blocks();
//6
        int line = 6;
        int counter = 0;


        telemetry.addData("started", isStarted());
        while (!isStarted() && !isStopRequested()) {
            blocks = huskyLens.blocks();
            counter++;
            telemetry.addData("amount of blocks", blocks.length);
            telemetry.addData("counter", counter);
            telemetry.addData("started", isStarted());
            telemetry.addData("Current Position", drive.pose.position);
            telemetry.addData("Current Heading", drive.pose.heading.toString());
            drive.updatePoseEstimate();

            telemetry.update();
            if (blocks.length != 0) {
                if (blocks[0].x < 110) {
                    // Prop is on left 4
                    line = 4;
                } else if (blocks[0].x > 220) {
                    // prop is on right 6
                    line = 6;
                } else {
                    // prop is on center 5
                    line = 5;
                }


                telemetry.addData("Line", line);
                telemetry.addData("Thing location  :", blocks[0].x);

                telemetry.update();
            }
//what should the preset be?
            if (blocks.length == 0) {
                line = 4;
            }

        }
        telemetry.addData("started after while", isStarted());

        waitForStart();
        telemetry.addData("Going for", line);
        telemetry.update();

        if (line == 4) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(-44, -36, Math.toRadians(0)), Math.toRadians(180))
                                    .build()));
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(-38.5, -29), Math.toRadians(180))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.45),
                            collector.collectorOffAction(),
                            new SleepAction(0.25)

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(0))
                                    .lineToXConstantHeading(-34)
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(-64.65, -5, Math.toRadians(180)), Math.toRadians(180))
                                    .setTangent(Math.toRadians(90))
                                    .strafeTo(new Vector2d(-63, -18))
                                    .build(),
                            collector.collectorStackInAction(),
                            transfer.transferInAction()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent((Math.toRadians(180)))
                                    .lineToXConstantHeading(-63)
                                    .build()
                    )
            );
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.75),
                            depositor.pixelHoldAction(),
                            new SleepAction(0.55)
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            collector.collectorStackOutAction(),
                            transfer.transferOutAction(),
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(-35, -6, Math.toRadians(180)), Math.toRadians(0))
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(0))
                                    .lineToXConstantHeading(30)
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            collector.collectorStackOffAction(),
                            transfer.transferOffAction(),
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(58, -34, Math.toRadians(180)), Math.toRadians(-90))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(2.3),
                            depositor.bottomPixelDropAction(),
                            new SleepAction(1.0)
                    )

            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(58, -47), Math.toRadians(-90))
                                    .build(),
                            new SleepAction(1.0),
                            depositor.topPixelDropAction(),
                            new SleepAction(1.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(1)
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(51, -8, Math.toRadians(180)), Math.toRadians(90))
                                    .build()
                    )
            );

        }
        if (line == 5) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(-34, -19, Math.toRadians(-90)), Math.toRadians(90))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.25),
                            collector.collectorOffAction(),
                            new SleepAction(.25)


                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-64.65, -5, Math.toRadians(180)), Math.toRadians(180))
                                    .setTangent(Math.toRadians(90))
                                    .strafeTo(new Vector2d(-63, -18))
                                    .build(),
                            collector.collectorStackInAction(),
                            transfer.transferInAction()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent((Math.toRadians(180)))
                                    .lineToXConstantHeading(-63)
                                    .build()
                    )
            );
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.15),
                            depositor.pixelHoldAction(),
                            new SleepAction(0.35)
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            collector.collectorStackOutAction(),
                            transfer.transferOutAction(),
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(-35, -6, Math.toRadians(180)), Math.toRadians(0))
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(0))
                                    .lineToXConstantHeading(30)
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            collector.collectorStackOffAction(),
                            transfer.transferOffAction(),
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(57.5, -34, Math.toRadians(180)), Math.toRadians(-90))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(2.3),
                            depositor.bottomPixelDropAction(),
                            new SleepAction(1.0)
                    )

            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(57, -41), Math.toRadians(-90))
                                    .build(),
                            new SleepAction(1.0),
                            depositor.topPixelDropAction(),
                            new SleepAction(1.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(1)
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(51, -8, Math.toRadians(180)), Math.toRadians(90))
                                    .build()
                    )
            );

        }

        if (line == 6) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(-34, -29, Math.toRadians(180)), Math.toRadians(0))
                                    .build()

                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(-28, -29), Math.toRadians(180))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.25),
                            collector.collectorOffAction(),
                            new SleepAction(.15)

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-64.65, -5, Math.toRadians(180)), Math.toRadians(180))
                                    .setTangent(Math.toRadians(90))
                                    .strafeTo(new Vector2d(-63, -18))
                                    .build(),
                            collector.collectorStackInAction(),
                            transfer.transferInAction()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent((Math.toRadians(180)))
                                    .lineToXConstantHeading(-63)
                                    .build()
                    )
            );
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.15),
                            depositor.pixelHoldAction(),
                            new SleepAction(0.35)
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            collector.collectorStackOutAction(),
                            transfer.transferOutAction(),
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(-35, -6, Math.toRadians(180)), Math.toRadians(0))
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(0))
                                    .lineToXConstantHeading(30)
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            collector.collectorStackOffAction(),
                            transfer.transferOffAction(),
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(57.5, -36, Math.toRadians(180)), Math.toRadians(-90))
                                    .build(), 
                            depositor.depositorScoringAction(),
                            new SleepAction(2.3),
                            depositor.bottomPixelDropAction(),
                            new SleepAction(1.0)
                    )

            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(57, -47), Math.toRadians(-90))
                                    .build(),
                            new SleepAction(1.0),
                            depositor.topPixelDropAction(),
                            new SleepAction(1.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(1)
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(51, -8, Math.toRadians(180)), Math.toRadians(90))
                                    .build()
                    )
            );
        }
    }
}