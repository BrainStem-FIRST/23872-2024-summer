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
import org.firstinspires.ftc.teamcode.robotAuto.TransferAuto;


@Autonomous(name="Blue Auto Backdrop April Tags", group="Robot")
public final class BlueAutoBackdropAprilTagsTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));



        telemetry.addData("started after while", isStarted());

        waitForStart();
        telemetry.update();



            //drive to where the robot can sense april tag
            Actions.runBlocking(
                    new SequentialAction(
                            drive.actionBuilder(drive.pose)
                                    .setReversed(false)
                                    .splineToConstantHeading(new Vector2d(-12, 5), Math.toRadians(0))
                                    .build()
                    )
            );

//


            //**sense tag and drive to it, but stop when you cant see it anymore, ready to do the next spline... but how?

            //drive to backdrop



        }}