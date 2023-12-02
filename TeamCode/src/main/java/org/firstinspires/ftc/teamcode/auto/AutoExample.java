package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name="Auto Test", group="Robot")
public final class AutoExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(36, -62, Math.toRadians(-90)));

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                    .setTangent(Math.toRadians(90))
                    .splineTo(new Vector2d(30, -36), Math.toRadians(0))
                    .build());
    }
}
