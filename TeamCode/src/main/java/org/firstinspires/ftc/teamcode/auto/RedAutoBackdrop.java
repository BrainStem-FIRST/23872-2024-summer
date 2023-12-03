package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.DepositorAuto;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Red Auto Backdrop", group="Robot")
public final class RedAutoBackdrop extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private HuskyLens.Block[] blocks;

    // Determine the prop position
    int targetTagPos = -1;
    int targetBlockPos = -1; // The block of interest within the blocks array.

    int getTargetTag(HuskyLens.Block[] blocks) {
//        telemetry.addData("ID 1", "True");
//        telemetry.addData("ID 2", "True");
//        updateTelemetry();

        int propPos;
        // for test purposes, return a known value
        // delete this segment when team prop is available
        //        return 1;
        telemetry.addData("Block List: ", blocks);
        if (blocks.length == 1) {
            if (blocks[0].x < 110) {
                // Prop is on left
                propPos = 4;
            } else if (blocks[0].x > 210) {
                // prop is on right
                propPos = 6;
            } else {
                // prop is on center
                propPos = 5;
            }
        } else {
            // could not recognize; return center
            propPos = 5;
        }

        return propPos;

    }


    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(-90)));
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */

        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        //huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();


//        if (!rateLimit.hasExpired()) {
//            continue;
//        }
//        rateLimit.reset();

        blocks = huskyLens.blocks();
        telemetry.addData("amount of blocks", blocks.length);
        double turn = 0;
        double yPos = -36;

        while (!isStarted()) {
            if (blocks.length != 0) {
                blocks = huskyLens.blocks();
                switch(blocks[0].x) {
                    case 4: {
                        turn = -90;
                        yPos = -32;
                        break;
                    }
                    case 5: {
                        turn = 0;
                        yPos = -36;
                        break;
                    }
                    case 6: {
                        turn = 90;
                        yPos = -40;
                        break;
                    }
                }

            } else {
                telemetry.addLine("Don't see the prop :(");

//                if (targetTagPos == -1) {
//                    telemetry.addLine("(The prop has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the prop before");
//                    telemetry.addData("which was: ", targetTagPos);
//                }

                sleep(20);
            }
            telemetry.addData("Y Position", yPos);
            telemetry.addData("Turn", turn);
            telemetry.update();
        }

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(90))
                                .splineTo(new Vector2d(10, -36), Math.toRadians(turn))
                                .build(),
                        collector.collectorOutAction(),
                        new SleepAction(0.8),
                        collector.collectorOffAction(),
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(0))
                                .splineTo(new Vector2d(55, yPos), Math.toRadians(-90))
                                .build(),
                        depositor.depositorScoringAction(),
                        new SleepAction(2.0),
                        depositor.pixelDropAction(),
                        new SleepAction(2.0),
                        depositor.depositorRestingAction(),
                        new SleepAction(2.0),
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(55, -55))
                                .build()

                )
        );
    }
}
