package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
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

@Autonomous(name="Blue Auto Audience Stacks", group="Robot")
public final class BlueAutoAudienceStacks extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private HuskyLens.Block[] blocks;

    private Pose2d startPose = new Pose2d(-35.25, 62.50, Math.toRadians(90));

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
//3
        int line = 3;
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
                    // Prop is on left 1
                    line = 1;
                } else if (blocks[0].x > 220) {
                    // prop is on right 3
                    line = 3;
                } else {
                    // prop is on center 2
                    line = 2;
                }


                telemetry.addData("Line", line);
                telemetry.addData("Thing location  :", blocks[0].x);

                telemetry.update();
            }
//2
            if (blocks.length == 0) {
                line = 3;
            }

        }
        telemetry.addData("started after while", isStarted());

        waitForStart();
        telemetry.addData("Going for", line);
        telemetry.update();

        if (line == 1) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(-44, 36, Math.toRadians(0)), Math.toRadians(180))
                                    .build()
                    )
            );

            telemetry.addData("Robot's position", drive.pose.position);

            telemetry.addData("Robot's heading", drive.pose.heading);

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(-31, 29, Math.toRadians(0)),Math.toRadians(0))                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.55),
                            collector.collectorOffAction(),
                            new SleepAction(0.2)

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .lineToXConstantHeading(-38)
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-67, 4, Math.toRadians(160)), Math.toRadians(180))
                                    .setTangent(Math.toRadians(-90))
                                    .strafeTo(new Vector2d(-65, 20))
                                    .build(),
                            collector.collectorInAction(),
                            transfer.transferInAction()

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent((Math.toRadians(180)))
                                    .lineToXConstantHeading(-64)
                                    .build()
                    )
            );
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(1.25),
                            depositor.pixelHoldAction(),
                            new SleepAction(1)
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
                                    .splineToLinearHeading(new Pose2d(-35, 9, Math.toRadians(180)), Math.toRadians(0))
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
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(45, 30, Math.toRadians(180)), Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(56, 35, Math.toRadians(180)), Math.toRadians(90))
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
                                    .splineToConstantHeading(new Vector2d(57.5, 47), Math.toRadians(90))
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
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(45, 8, Math.toRadians(180)), Math.toRadians(-90))
                                    .build()
                    )
            );

        }
        if (line == 2) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(-34, 20, Math.toRadians(90)), Math.toRadians(-90))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.55),
                            collector.collectorOffAction(),
                            new SleepAction(.25)


                    )
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .lineToYConstantHeading(12)
                                    .build()
                    )
                    );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            collector.collectorInAction(),
                            transfer.transferInAction(),
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-67, 3, Math.toRadians(160)), Math.toRadians(180))
                                    .setTangent(Math.toRadians(-90))
                                    .strafeTo(new Vector2d(-65.75, 19))
                                    .build()

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent((Math.toRadians(180)))
                                    .lineToXConstantHeading(-64)
                                    .build()
                    )
            );
            Actions.runBlocking(
                    new SequentialAction(
//                            new SleepAction(0.25),
                            depositor.pixelHoldAction(),
                            new SleepAction(0.38)
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
                                    .splineToLinearHeading(new Pose2d(-35, 6, Math.toRadians(180)), Math.toRadians(0))
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
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(90))
                                    .build(),
                            depositor.depositorScoringAction(),
                            drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading(new Pose2d(54.5, 43, Math.toRadians(180)), Math.toRadians(0))
                                    .build(),
                            depositor.bottomPixelDropAction(),
                            new SleepAction(1.0)
                    )

            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(56.5, 33), Math.toRadians(90))
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
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(51, 12, Math.toRadians(180)), Math.toRadians(-90))
                                    .build()
                    )
            );

        }

        if (line == 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(-34, 29, Math.toRadians(180)), Math.toRadians(0))
                                    .build()

                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(-39.5, 29), Math.toRadians(180))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.55),
                            collector.collectorOffAction(),
                            new SleepAction(.15)

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(0))
                                    .lineToXConstantHeading(-35)
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(-67, 2, Math.toRadians(165)), Math.toRadians(180))
                                    .setTangent(Math.toRadians(-90))
                                    .strafeTo(new Vector2d(-65.75, 20))
                                    .build(),
                            collector.collectorInAction(),
                            transfer.transferInAction()

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent((Math.toRadians(180)))
                                    .lineToXConstantHeading(-64)
                                    .build()
                    )
            );
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.25),
                            depositor.pixelHoldAction(),
                            new SleepAction(1)
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
                                    .splineToLinearHeading(new Pose2d(-35, 8, Math.toRadians(180)), Math.toRadians(0))
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
                                    .splineToLinearHeading(new Pose2d(50, 30, Math.toRadians(180)), Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(57, 44.5, Math.toRadians(180)), Math.toRadians(90))
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
                                    .splineToConstantHeading(new Vector2d(58, 26), Math.toRadians(90))
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
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(51, 8, Math.toRadians(180)), Math.toRadians(-90))
                                    .build()
                    )
            );
        }
    }
}