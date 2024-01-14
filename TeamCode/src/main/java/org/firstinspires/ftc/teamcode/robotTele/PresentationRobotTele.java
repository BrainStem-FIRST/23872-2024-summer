package org.firstinspires.ftc.teamcode.robotTele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PresentationRobotTele {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public CollectorTele collector;
    public TransferTele transfer;
    public DepositorTele depositor;
    public LiftTele lift;
    public HangingTele hanging;
    public DroneTele drone;

    public PresentationRobotTele(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;


        collector = new CollectorTele(hardwareMap, telemetry);
        hanging = new HangingTele(hardwareMap, telemetry);
        drone = new DroneTele(hardwareMap, telemetry);
        transfer = new TransferTele(hardwareMap, telemetry);
        depositor = new DepositorTele(hardwareMap, telemetry);
        lift = new LiftTele(hardwareMap, telemetry);
    }

    public void update() {
        telemetry.addData("collectorState", collector.collectorState);
        collector.setCollectorState();
        hanging.hangingState();
        drone.setServoState();
        transfer.transferState();
        depositor.depositorServoState(lift);
        lift.liftState();
        lift.updateLevel();
    }

    }


