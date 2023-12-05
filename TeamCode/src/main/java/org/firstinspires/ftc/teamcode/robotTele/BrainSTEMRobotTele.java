package org.firstinspires.ftc.teamcode.robotTele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BrainSTEMRobotTele {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public CollectorTele collector;
    public TransferTele transfer;
    public DepositorTele depositor;
    public LiftTele lift;
<<<<<<< HEAD
//    public HangingTele hanging;
=======
    public HangingTele hanging;
    public DroneTele drone;
>>>>>>> 68230b3f49e3c8dcf7c3a9bdd92ad1265dd4e869

    public BrainSTEMRobotTele(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;


        collector = new CollectorTele(hardwareMap, telemetry);
<<<<<<< HEAD
//        hanging = new HangingTele(hardwareMap, telemetry);
=======
        hanging = new HangingTele(hardwareMap, telemetry);
        drone = new DroneTele(hardwareMap, telemetry);
>>>>>>> 68230b3f49e3c8dcf7c3a9bdd92ad1265dd4e869
        transfer = new TransferTele(hardwareMap, telemetry);
        depositor = new DepositorTele(hardwareMap, telemetry);
        lift = new LiftTele(hardwareMap, telemetry);
    }

    public void update() {
        telemetry.addData("collectorState", collector.collectorState);
        collector.setCollectorState();
<<<<<<< HEAD
//        hanging.hangingState();
//        hanging.setServoState();
=======
        hanging.hangingState();
        hanging.setServoState();
        drone.setServoState();
>>>>>>> 68230b3f49e3c8dcf7c3a9bdd92ad1265dd4e869
        transfer.transferState();
        depositor.depositorServoState(lift);
        depositor.pixelState();
        lift.liftState();
        lift.updateLevel();
    }

    }


