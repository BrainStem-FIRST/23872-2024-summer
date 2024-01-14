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

@Autonomous(name="Blue Auto Audience", group="Robot")
public final class BlueAutoAudience extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private HuskyLens.Block[] blocks;

    private Pose2d startPose = new Pose2d(-35.25, 62.50, Math.toRadians(90));

    // Determine the prop position
    int targetTagPos = -1;
    int targetBlockPos = -1; // The block of interest within the blocks array.

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        //Set starting position
        drive.pose = startPose;
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);

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
                if (blocks[0].x < 80) {
                    // Prop is on left 1
                    line = 1;
                } else if (blocks[0].x > 240) {
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
            if (blocks.length == 0){
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
                                    .build()));
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(0))
                                    .splineToConstantHeading(new Vector2d(-30.75, 29), Math.toRadians(0))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.85),
                            collector.collectorOffAction()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-35, 6, Math.toRadians(0)), Math.toRadians(0))
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
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(54.25, 42.75, Math.toRadians(180)), Math.toRadians(90))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(1.0),
                            depositor.pixelDropAction(),
                            new SleepAction(1.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(1.5)
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading( new Pose2d(51,8, Math.toRadians(180)), Math.toRadians(-90))
                                    .build()
                    )
            );

            while (opModeIsActive()) {
                telemetry.addData("Heading", drive.pose.heading);
                telemetry.update();
            }
        }
        if (line == 2) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(-34, 19, Math.toRadians(90)), Math.toRadians(-90))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(1),
                            collector.collectorOffAction()

                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(-90))
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

            // Move robot to backdrop

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(52.5, 38, Math.toRadians(180)), Math.toRadians(0))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(1.0),
                            depositor.pixelDropAction(),
                            new SleepAction(1.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(1.5)



                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading( new Pose2d(51,9, Math.toRadians(180)), Math.toRadians(-90))
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
                                    .splineToConstantHeading(new Vector2d(-39, 29), Math.toRadians(180))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.75),
                            collector.collectorOffAction()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(0))
                                    .lineToXConstantHeading(-35.5)
                                    .build()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .strafeTo(new Vector2d(-34,6))
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
//            // Move robot to backdrop
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(55, 33, Math.toRadians(180)), Math.toRadians(0))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(1.0),
                            depositor.pixelDropAction(),
                            new SleepAction(1.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(1.5)



                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading( new Pose2d(51,7.5, Math.toRadians(180)), Math.toRadians(-90))
                                    .build()
                    )
            );
        }
    }
}



