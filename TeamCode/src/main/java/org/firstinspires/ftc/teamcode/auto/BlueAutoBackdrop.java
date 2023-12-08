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

@Autonomous(name="Blue Auto Backdrop", group="Robot")
public final class BlueAutoBackdrop extends LinearOpMode {

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

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");


        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();


        blocks = huskyLens.blocks();

        double turn = 0;
        double yPos = -36;
        int line = 4;
        int counter = 0;

        while (!isStarted() && !isStopRequested()) {
            blocks = huskyLens.blocks();
            telemetry.addData("amount of blocks", blocks.length);
            telemetry.addData("counter", counter);
            counter ++;
//            telemetry.update();
            if (blocks.length != 0) {
                if (blocks[0].x < 50) {
                    // Prop is on left
                    line = 4;
                } else if (blocks[0].x > 280) {
                    // prop is on right
                    line = 6;
                } else if (blocks[0].x > 50 && blocks[0].x < 280) {
                    // prop is on center
                    line = 5;


                    switch (line) {
                        case 4: {
                            turn = 0;
                            yPos = 28;
                            break;
                        }
                        case 5: {
                            turn = -90;
                            yPos = 32;
                            break;
                        }
                        case 6: {
                            turn = 180;
                            yPos = 40;
                            break;
                        }
                    }

                } else {
                    telemetry.addLine("Don't see the prop");

                    sleep(20);
                }
                telemetry.addData("Line", line);
                telemetry.addData("Y Position", yPos);
                telemetry.addData("Turn", turn);
                telemetry.addData("Thing location  :", blocks[0].x);

                telemetry.update();
            }

            waitForStart();

            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(90))
                                    .splineTo(new Vector2d(10, 35), Math.toRadians(180))
                                    .build(),
                            collector.collectorOutAction(),
                            new SleepAction(0.65),
                            collector.collectorOffAction(),
                            drive.actionBuilder(drive.pose)
                                    .setTangent(Math.toRadians(180))
                                    .splineTo(new Vector2d(55, 36), Math.toRadians(90))
                                    .build(),
                            depositor.depositorScoringAction(),
                            new SleepAction(2.0),
                            depositor.pixelDropAction(),
                            new SleepAction(2.0),
                            depositor.depositorRestingAction(),
                            new SleepAction(2.0),
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(58, 65))
                                    .build()
                    )
            );
        }
    }
}
