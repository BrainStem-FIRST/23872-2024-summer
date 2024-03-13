//package org.firstinspires.ftc.teamcode.auto;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.TranslationalVelConstraint;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
//import org.firstinspires.ftc.teamcode.robotAuto.DepositorAuto;
//import org.firstinspires.ftc.teamcode.robotAuto.TransferAuto;
//
//@Autonomous(name="Red Auto Backdrop Stacks", group="Robot")
//public final class RedAutoBackdropStacks extends LinearOpMode {
//
//    private final int READ_PERIOD = 1;
//    private HuskyLens huskyLens;
//    private HuskyLens.Block[] blocks;
//
//    // Determine the prop position
//    int targetTagPos = -1;
//    int targetBlockPos = -1; // The block of interest within the blocks array.
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(-85)));
//        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
//        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);
//        TransferAuto transfer = new TransferAuto(hardwareMap, telemetry);
//
//
//        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
//
//
//        if (!huskyLens.knock()) {
//            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
//        } else {
//            telemetry.addData(">>", "Press start to continue");
//        }
//
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
//
//        telemetry.update();
//
//
//        blocks = huskyLens.blocks();
//
//        int line = 6;
//        int counter = 0;
//
//
//        telemetry.addData("started", isStarted());
//        while (!isStarted() && !isStopRequested()) {
//            blocks = huskyLens.blocks();
//            counter++;
//            telemetry.addData("amount of blocks", blocks.length);
//            telemetry.addData("counter", counter);
//            telemetry.addData("started", isStarted());
//
//            telemetry.update();
////            if (blocks.length != 0)
////            {
////                if (blocks[0].x < 80) {
////                    // Prop is on left
////                    line = 4;
////                } else if (blocks[0].x > 240) {
////                    // prop is on right
////                    line = 6;
////                } else {
////                    // prop is on center
////                    line = 5;
////                }
//
//            if (blocks.length != 0)
//            {
//                if (blocks[0].x < 170) {
//                    // Prop is on left
//                    line = 4;
//                } else if (blocks[0].x > 170) {
//                    // prop is on right
//                    line = 5;
//                }
//
//
//                telemetry.addData("Line", line);
//                telemetry.addData("Thing location  :", blocks[0].x);
//
//                telemetry.update();
//            }
//            //6
//            if (blocks.length == 0){
//                line = 6;
//            }
//
//        }
//        telemetry.addData("started after while", isStarted());
//
//        waitForStart();
//        telemetry.addData("Going for", line);
//        telemetry.update();
//
//
//
//
//
//        if (line == 4) {
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(90))
//                                    .splineToLinearHeading(new Pose2d(1, -27, Math.toRadians(180)), Math.toRadians(180), new TranslationalVelConstraint(45))
//                                    .build(),
//                            collector.collectorOutAction(),
//                            new SleepAction(0.7),
//                            collector.collectorOffAction()
//                            // Move robot to backdrop
//                    )
//            );
//
//                            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(0))
//                                    .splineToLinearHeading(new Pose2d(56.5, -24, Math.toRadians(180)), Math.toRadians(0))
//                                    .build(),
//                            depositor.depositorScoringAction(),
//                            new SleepAction(1.5),
//                            depositor.pixelDropAction(),
//                            new SleepAction(1.0),
//                            depositor.depositorRestingAction(),
//                            new SleepAction(1.5)
//                            )
//            );
//
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setTangent(Math.toRadians(90))
//                                    .splineToLinearHeading(new Pose2d(-65, -11, Math.toRadians(-175)), Math.toRadians(180))
//                                    .build(),
//                            collector.collectorStackInAction(),
//                            transfer.transferInAction()
//                    )
//            );
//
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setTangent(Math.toRadians(90))
//                                    .strafeTo(new Vector2d(-66, -27))
//                                    .build()
//                    )
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new SleepAction(1.25),
//                            depositor.pixelHoldAction(),
//                            new SleepAction(2)
//                    )
//            );
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            collector.collectorStackOutAction(),
//                            transfer.transferOutAction(),
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(-60))
//                                    .splineToLinearHeading( new Pose2d(-12,-10, Math.toRadians(180)), Math.toRadians(-40),new TranslationalVelConstraint(45))
//                                    .splineToConstantHeading(new Vector2d(57,-44), Math.toRadians(-40), new TranslationalVelConstraint(35))
//                                    .build(),
//                            depositor.depositorScoringAction(),
//                            new SleepAction(1.5),
//                            depositor.pixelDropAction(),
//                            new SleepAction(1.5),
//                            depositor.depositorRestingAction(),
//                            new SleepAction(1.5),
//                            collector.collectorStackOffAction(),
//                            transfer.transferOffAction()
//
//                    )
//            );
//
//
//        }
//
//        if (line == 5) {
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setTangent(Math.toRadians(90))
//                                    .splineToLinearHeading(new Pose2d(15, -28, Math.toRadians(90)), Math.toRadians(90))
//                                    .build(),
//                            collector.collectorOutAction(),
//                            new SleepAction(0.60),
//                            collector.collectorOffAction()
//                    )
//            );
//            // Move robot to backdrop
//
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(-90))
//                                    .splineToLinearHeading(new Pose2d(58, -30, Math.toRadians(180)), Math.toRadians(0))
//                                    .build(),
//                            depositor.depositorScoringAction(),
//                            new SleepAction(1.0),
//                            depositor.pixelDropAction(),
//                            new SleepAction(1.0),
//                            depositor.depositorRestingAction(),
//                            new SleepAction(1.0)
//                    )
//            );
//
//
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setTangent(Math.toRadians(90))
//                                    .splineToLinearHeading(new Pose2d(-65,-10, Math.toRadians(180)), Math.toRadians(180))
//                                    .build(),
//                            collector.collectorStackInAction(),
//                            transfer.transferInAction()
//                    )
//            );
//
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setTangent(Math.toRadians(90))
//                                    .strafeTo(new Vector2d(-66, -25))
//                                    .build()
//                    )
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new SleepAction(1.25),
//                            depositor.pixelHoldAction(),
//                            new SleepAction(2)
//                    )
//            );
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            collector.collectorStackOutAction(),
//                            transfer.transferOutAction(),
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(-60))
//                                    .splineToLinearHeading( new Pose2d(-12,-10, Math.toRadians(180)), Math.toRadians(-40),new TranslationalVelConstraint(45))
//                                    .splineToConstantHeading(new Vector2d(57,-34), Math.toRadians(-40), new TranslationalVelConstraint(35))
//                                    .build(),
//                            depositor.depositorScoringAction(),
//                            new SleepAction(1.5),
//                            depositor.pixelDropAction(),
//                            new SleepAction(1.5),
//                            depositor.depositorRestingAction(),
//                            new SleepAction(1.5),
//                            collector.collectorStackOffAction(),
//                            transfer.transferOffAction()
//
//                    )
//            );
//        }
//
//        if (line == 6) {
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(0))
//                                    .splineToLinearHeading(new Pose2d(29, -26, Math.toRadians(-180)),Math.PI/2)
//                                    .build(),
//                            collector.collectorOutAction(),
//                            new SleepAction(.67),
//                            collector.collectorOffAction()
//                    )
//            );
//
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(-90))
//                                    .splineToLinearHeading(new Pose2d(55.75, -43, Math.toRadians(180)), Math.toRadians(0))
//                                    .build(),
//                            depositor.depositorScoringAction(),
//                            new SleepAction(1.0),
//                            depositor.pixelDropAction(),
//                            new SleepAction(1.0),
//                            depositor.depositorRestingAction(),
//                            new SleepAction(1)
//                    )
//            );
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setTangent(Math.toRadians(90))
//                                    .splineToLinearHeading(new Pose2d(-65, -11, Math.toRadians(180)), Math.toRadians(180))
//                                    .build(),
//                            collector.collectorStackInAction(),
//                            transfer.transferInAction()
//                    )
//            );
//
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            drive.actionBuilder(drive.pose)
//                                    .setTangent(Math.toRadians(90))
//                                    .strafeTo(new Vector2d(-66, -27))
//                                    .build()
//                    )
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new SleepAction(1.25),
//                            depositor.pixelHoldAction(),
//                            new SleepAction(2)
//                    )
//            );
//            drive.updatePoseEstimate();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            collector.collectorStackOutAction(),
//                            transfer.transferOutAction(),
//                            drive.actionBuilder(drive.pose)
//                                    .setReversed(true)
//                                    .setTangent(Math.toRadians(-60))
//                                    .splineToLinearHeading( new Pose2d(-12,-12, Math.toRadians(180)), Math.toRadians(-40),new TranslationalVelConstraint(45))
//                                    .splineToConstantHeading(new Vector2d(57,-34), Math.toRadians(-40), new TranslationalVelConstraint(35))
//                                    .build(),
//                            depositor.depositorScoringAction(),
//                            new SleepAction(1.5),
//                            depositor.pixelDropAction(),
//                            new SleepAction(1.5),
//                            depositor.depositorRestingAction(),
//                            new SleepAction(1.5),
//                            collector.collectorStackOffAction(),
//                            transfer.transferOffAction()
//
//                    )
//            );
//
//        }
//    }
//}
//
//
//
