package org.firstinspires.ftc.teamcode.summerCamp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.DepositorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.TransferAuto;
import org.firstinspires.ftc.teamcode.robotTele.DepositorTele;

import java.util.Vector;

@Autonomous(name="Blue Auto", group="Robot")
public final class BlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(36, -60, Math.toRadians(90)));
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);
        TransferAuto transfer = new TransferAuto(hardwareMap, telemetry);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(25,-24), Math.toRadians(90))
                                .build(),
                        collector.collectorInAction(),
                        transfer.transferInAction(),
                        new SleepAction(2),
                        depositor.pixelHoldAction(),
                        collector.collectorOffAction(),
                        transfer.transferOffAction()));
/*
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(30,-30), Math.toRadians(0))
                                .build(),
                        depositor.depositorScoringAction(),
                        new SleepAction(1.0),
                        depositor.pixelDropAction(),
                        new SleepAction(1.0),
                        depositor.depositorRestingAction(),
                        new SleepAction(1.0)));
   */     }

}
