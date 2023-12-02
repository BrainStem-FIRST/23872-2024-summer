package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {

    private DcMotorEx liftMotor;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;


    public LiftState liftState = LiftState.ZERO;

    private final static int levelZeroHeight = 0;
    private final static int levelOneHeight = 100;
    private final static int levelTwoHeight = 200;
    private final static int levelThreeHeight = 300;
    private final static int levelFourHeight = 400;
    private final static int levelFiveHeight = 500;
    private final static int levelSixHeight = 600;
    private final static int levelSevenHeight = 700;
    private final static int levelEightHeight = 800;

    public void setStateLevelZero(){
        liftState = LiftState.ZERO;
    }
    public void setStateLevelThree(){
        liftState = LiftState.THREE;
    }

    public Lift(HardwareMap hardwareMap, Telemetry telemetry){
        this.liftMotor = liftMotor;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;


        liftMotor = new org.firstinspires.ftc.teamcode.util.CachingMotor(hardwareMap.get(DcMotorEx.class, "Lift"));
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    public enum LiftState {
      ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT
    }

   public void liftState() {
        switch (liftState) {
            case ZERO: {
                setLiftHeight(levelZeroHeight);
                break;
            }
            case ONE: {
                setLiftHeight(levelOneHeight);
                break;
            }
            case TWO: {
                setLiftHeight(levelTwoHeight);
                break;
            }
            case THREE: {
                setLiftHeight(levelThreeHeight);
                break;
            }
            case FOUR: {
                setLiftHeight(levelFourHeight);
                break;
            }
            case FIVE: {
                setLiftHeight(levelFiveHeight);
                break;
            }
            case SIX: {
                setLiftHeight(levelSixHeight);
                break;
            }
            case SEVEN: {
                setLiftHeight(levelSevenHeight);
                break;
            }
            case EIGHT: {
                setLiftHeight(levelEightHeight);
                break;
            }
        }
    }

    public void setLiftHeight(int liftHeight) {
        liftMotor.setPower(0.3);
    }

    public void setLiftUp() {
        liftMotor.setPower(0.6);
    }

    public void setLiftDown() {
        liftMotor.setPower(-0.6);
    }

    public void setLiftOff() {
        liftMotor.setPower(0);
    }




}
