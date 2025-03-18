package Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ArmV2 {
    DcMotorEx motor;
    PIDController pid;
    public static double kP = 0.008;
    public static double kI = 0;
    public static double kD = 0;
    public static int posDown = -823;
    public static int posClip = -424;
    public static int posBase = -20;
    public static int posHook = -424;
    public static int posBasket = 119;
    public static int posSpecimen = 119;
    static int targetPosition;
    public static double ff = .15;

    static double degPerTick = 0.17241379;

    static double shoulder_start_angle = 128;

    public ArmV2(@NonNull HardwareMap hwMap){
        motor = hwMap.get(DcMotorEx.class,"shoulderMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pid = new PIDController(kP, kI, kD);
    }
    public void update(){
        pid.setPID(kP, kI, kD);
        double output = pid.calculate(motor.getCurrentPosition(), targetPosition);

        motor.setPower(output +ff*Math.cos(Math.toRadians(shoulderAngle())));
    }

    public void setPosition(int i){
        targetPosition = i;
    }
    public void setPosition_PickUp(){
        targetPosition = posDown;
    }
    public void setPosition_SpecimenPickUp(){
        targetPosition = posSpecimen;
    }
    public void setPosition_Hook(){
        targetPosition = posHook;
    }
    public void setPosition_Basket(){
        targetPosition = posBasket;
    }
    public void setPositionBase(){
        targetPosition = posBase;
    }
    public double shoulderAngle(){
        return shoulder_start_angle + (motor.getCurrentPosition()*degPerTick);
    }

    public int getPosition(){
        return motor.getCurrentPosition();
    }
    public double getPower(){
        return motor.getPower();
    }


    public void manualMove(double input){
        motor.setPower((input/2.5)+ff*Math.cos(shoulderAngle()));
    }
}
