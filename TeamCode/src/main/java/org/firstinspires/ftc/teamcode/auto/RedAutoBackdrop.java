package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ParallelAction;
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
    int targetBlockPos = -1; // The block of interest within the blocks array.e
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(-90)));
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


        double turn = 0;
        double yPos = -30;
        int line = 4;
        int cnt = 0;
        int tan = 0;
        int bkdturn = 0;

        telemetry.addData("started", isStarted());
        while (!isStarted() && !isStopRequested()) {
            blocks = huskyLens.blocks();
            cnt++;
            telemetry.addData("amount of blocks", blocks.length);
            telemetry.addData("counter", cnt);
            telemetry.addData("started", isStarted());

            telemetry.update();
            if (blocks.length != 0) {
                if (blocks[0].x < 50) {
                    // Prop is on left
                    line = 4;
                } else if (blocks[0].x > 270) {
                    // prop is on right
                    line = 6;
                } else  {
                    // prop is on center
                    line = 5;
                }

                switch (line) {
                    case 4: {
                        turn = 90;
                        yPos = -30;
                        tan = 180;
                        bkdturn = 0;
                        break;
                    }
                    case 5: {
                        turn = 180;
                        yPos = -38;
                        tan = 0;
                        bkdturn = 90;
                        break;
                    }
                    case 6: {
                        turn = 90;
                        yPos = -45;
                        tan = 0;
                        bkdturn = 180;
                        break;
                    }

                }
                telemetry.addData("Line", line);
                telemetry.addData("Y Position", yPos);
                telemetry.addData("Turn", turn);
                telemetry.addData("Back Turn", bkdturn);
                telemetry.addData("Thing location :", blocks[0].x);
                telemetry.addData("started", isStarted());

                telemetry.update();
            }

        }
        telemetry.addData("started after while", isStarted());

        waitForStart();
        telemetry.addData("Going for", line);
        telemetry.update();
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(tan))
                                .splineTo(new Vector2d(14, -30), Math.toRadians(turn))
                                .build(),
                        collector.collectorOutAction(),
                        new SleepAction(0.8),
                        collector.collectorOffAction(),
// Pull robot back to clear the spikes

                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(-90))
                                .splineTo(new Vector2d(14, -55), Math.toRadians(-90))
                                .build(),
// Move robot to backdrop

                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(0))
                                .splineTo(new Vector2d(62, yPos), Math.toRadians(bkdturn))
                                .build(),
                        depositor.depositorScoringAction(),
                        new SleepAction(2.0),
                        depositor.pixelDropAction(),
                        new SleepAction(2.0),
                        depositor.depositorRestingAction(),
                        new SleepAction(2.0),
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(-90))
                                .strafeTo(new Vector2d(58, -65))
                                .build()
                )
        );
    }
}




