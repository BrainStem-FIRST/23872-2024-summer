package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.DepositorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.TransferAuto;


@Autonomous(name="Blue Auto Backdrop Stacks", group="Robot")
public final class BlueAutoBackdropStacks extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private HuskyLens.Block[] blocks;

    // Determine the prop position
    int targetTagPos = -1;
    int targetBlockPos = -1; // The block of interest within the blocks array.

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, 62, Math.toRadians(90)));
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

        int line = 3;
        int counter = 0;


        telemetry.addData("started", isStarted());
        while (!isStarted() && !isStopRequested()) {
            blocks = huskyLens.blocks();
            counter++;
            telemetry.addData("amount of blocks", blocks.length);
            telemetry.addData("counter", counter);
            telemetry.addData("started", isStarted());

            telemetry.update();
            if (blocks.length != 0) {
                if (blocks[0].x < 80) {
                    // Prop is on left
                    line = 1;
                } else if (blocks[0].x > 240) {
                    // prop is on right
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
                line = 1;
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
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(23, 26, Math.toRadians(-60)),-90, new TranslationalVelConstraint(45))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(.67),
                            collector.collectorOffAction()
                    )
            );

            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(55.75, 42, Math.toRadians(180)), Math.toRadians(0))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(0.75),
                            depositor.topPixelDropAction(),
                            new SleepAction(0.75),
                            depositor.depositorRestingAction(),
                            new SleepAction(0.7  )
                    )
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .splineToLinearHeading( new Pose2d(-63,5, Math.toRadians(180)), Math.toRadians(-90),new TranslationalVelConstraint(45))
                                    .build(),
                            collector.collectorInAction(),
                            transfer.transferInAction()
                    ));



            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .strafeTo(new Vector2d(-63, 19), new TranslationalVelConstraint(45))
                                    .build()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.45),
                            depositor.pixelHoldAction(),
                            new SleepAction(1),
                            collector.collectorOutAction(),
                            transfer.transferOutAction()
                    )
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(60))
                                    .splineToLinearHeading( new Pose2d(-12,8, Math.toRadians(180)), Math.toRadians(90),new TranslationalVelConstraint(45))
                                    .splineToConstantHeading(new Vector2d(55,40), Math.toRadians(60), new TranslationalVelConstraint(45))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(0.5),
                            depositor.pixelDropAction(),
                            new SleepAction(0.5),
                            depositor.depositorRestingAction(),
                            new SleepAction(0.5),
                            collector.collectorOffAction(),
                            transfer.transferOffAction()

                    )
            );

        }
        if (line == 2) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(15, 29, Math.toRadians(-90)), Math.toRadians(-90))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(0.65),
                            collector.collectorOffAction()
                            // Move robot to backdrop
                    )
            );
                            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(53, 33, Math.toRadians(180)), Math.toRadians(0))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(2.0),
                            depositor.pixelDropAction(),
                            new SleepAction(2.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(2.5)
                    )
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading( new Pose2d(48,65, Math.toRadians(180)), Math.toRadians(90))
                                    .build()
                    )
            );
        }
        if (line == 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(5, 28.5, Math.toRadians(180)), Math.toRadians(180))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(0.65),
                            collector.collectorOffAction()
                            // Move robot to backdrop
                    )
            );
            drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(true)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(53, 24.75, Math.toRadians(180)), Math.toRadians(0))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(2.0),
                            depositor.pixelDropAction(),
                            new SleepAction(2.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(2.5)
                            )
            ); drive.updatePoseEstimate();
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading( new Pose2d(48,65, Math.toRadians(180)), Math.toRadians(90))
                                    .build()

                    )
            );
        }
    }
}



