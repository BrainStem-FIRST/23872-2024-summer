package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
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
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(-90)));
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        waitForStart();

//        new SequentialAction(
//                drive.actionBuilder(drive.pose)
//                        .setTangent(Math.toRadians(90))
//                        .splineTo(new Vector2d(30, -36), Math.toRadians(0))
//                        .build(),

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(90))
                                .splineTo(new Vector2d(12, -36), Math.toRadians(0))
                                .build(),
                        collector.collectorOutAction(),
                        new SleepAction(1.0),
                        collector.collectorOffAction(),
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(0))
                                .splineTo(new Vector2d(50, -36), Math.toRadians(180))
                                .build()
                )
        );
    }
}
