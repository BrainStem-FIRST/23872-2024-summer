package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.DepositorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.TransferAuto;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="Auto Test", group="Robot")
public final class AutoExample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -62, Math.toRadians(-90)));
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);




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
                                .splineTo(new Vector2d(10, -36), Math.toRadians(0))
                                .build(),
                        collector.collectorOutAction(),
                        new SleepAction(0.8),
                        collector.collectorOffAction(),
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(0))
                                .splineTo(new Vector2d(55, -37), Math.toRadians(-90))
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
