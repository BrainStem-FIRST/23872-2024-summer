package org.firstinspires.ftc.teamcode.robotAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftAuto {
    public DcMotorEx liftMotor;
    private HardwareMap hardwareMap;
    private final Telemetry telemetry;
    public LiftState liftState = LiftState.ZERO;
    public int levelCounter = 0;



    private final static double kP = 0.85;
    //kP value not working.. needs tuning
    private final static double kI = 0.0;
    private final static double kD = 0.0;
    private final static double kS = 0.002;
    PIDController pidController = new PIDController(kP, kI, kD);
    private final static int levelZeroHeight = 0;
    private final static int levelOneHeight = 108;
    private final static int levelTwoHeight = 326;
    private final static int levelThreeHeight = 553;
    private final static int levelFourHeight = 788;
    private final static int levelFiveHeight = 1000;

    public LiftAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        pidController.setInputBounds(0, 1000);
        pidController.setOutputBounds(-1, 1);
        liftMotor = new org.firstinspires.ftc.teamcode.util.CachingMotor(hardwareMap.get(DcMotorEx.class, "Lift"));
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public enum LiftState {
        ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT
    }

    public void liftState() {

        switch (liftState) {
            case ZERO: {
                setLiftHeight(levelZeroHeight);
                updateLevel();
                telemetry.addData("lift enum", "Zero");
                break;
            }
            case ONE: {
                setLiftHeight(levelOneHeight);
                updateLevel();
                telemetry.addData("lift enum", "One");
                break;
            }
            case TWO: {
                setLiftHeight(levelTwoHeight);
                updateLevel();
                telemetry.addData("lift enum", "Two");
                break;
            }
            case THREE: {
                setLiftHeight(levelThreeHeight);
                updateLevel();
                telemetry.addData("lift enum", "Three");
                break;
            }
            case FOUR: {
                setLiftHeight(levelFourHeight);
                updateLevel();
                telemetry.addData("lift enum", "Four");
                break;
            }
            case FIVE: {
                setLiftHeight(levelFiveHeight);
                updateLevel();
                telemetry.addData("lift enum", "Five");
                break;
            }
        }
    }
            public void setLiftHeight(int liftHeight) {
                pidController.setTarget(liftHeight);
                double error = liftHeight - liftMotor.getCurrentPosition();
                if (Math.abs(error) < 5){
                    error = 0;
                }
                double power = pidController.updateWithError(error) + kS;
                if (Math.abs(error) < 25){
                    power = power/4;
                }
                liftMotor.setPower(power);
                telemetry.addData("liftError", error);
                telemetry.addData("liftPower", power);
                telemetry.addData("liftTarget", liftHeight);

            }

            public void increaseLevel(){
                levelCounter += 1;
                if (levelCounter >= 6){
                    levelCounter = 5;
                }
            }
            public void decreaseLevel() {
                levelCounter -= 1;
                if (levelCounter < 0) {
                    levelCounter = 0;
                }
            }
            public void updateLevel(){
                telemetry.addData("Level Counter", levelCounter);
                switch (levelCounter){
                    case 0:
                        liftState = LiftState.ZERO;
                        break;
                    case 1:
                        liftState = LiftState.ONE;
                        break;
                    case 2:
                        liftState = LiftState.TWO;
                        break;
                    case 3:
                        liftState = LiftState.THREE;
                        break;
                    case 4:
                        liftState = LiftState.FOUR;
                        break;
                    case 5:
                        liftState = LiftState.FIVE;
                        break;
                }
            }
            public void setLiftOff () {
        liftMotor.setPower(0);
    }

    public void setRawPower (double power){
        liftMotor.setPower(power);
    }

    public void setLiftZero () {
        liftState = LiftState.ZERO;
    }

    public void setLiftOne () {
        liftState = LiftState.ONE;
    }
    public void setLiftTwo() {
        liftState = LiftState.TWO;
    }
    public void setLiftThree() {
        liftState = LiftState.THREE;
    }
    public void setLiftFour() {
        liftState = LiftState.FOUR;
    }
    public void setLiftFive() {
        liftState = LiftState.FIVE;
    }

}