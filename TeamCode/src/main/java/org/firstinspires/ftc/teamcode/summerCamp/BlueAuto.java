package org.firstinspires.ftc.teamcode.summerCamp;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotAuto.CollectorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.DepositorAuto;
import org.firstinspires.ftc.teamcode.robotAuto.TransferAuto;

import java.util.Vector;

@Autonomous(name="Blue Auto", group="Robot")
public final class BlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(36, -60,Math.toRadians(0)));
        CollectorAuto collector = new CollectorAuto(hardwareMap, telemetry);
        DepositorAuto depositor = new DepositorAuto(hardwareMap, telemetry);
        TransferAuto transfer = new TransferAuto(hardwareMap, telemetry);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(drive.pose)
                                .setReversed(true)
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(36,-36), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(42,-30), Math.toRadians(0))
                                .build()));
    }

}
