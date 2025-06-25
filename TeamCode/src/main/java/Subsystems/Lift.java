package Subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    private DcMotorEx liftMotorR;
    private DcMotorEx liftMotorL;

    private int lift_target = 0;
    public static int lift_target_change = 0;
    public static int lift_max_position = 3735;
    private static PIDController lift_controller;
    public static double lift_kP = 0.01;
    public static double lift_kI = 0.0;
    public static double lift_kD = 0.0;
    public static double lift_kH = 0.00;

    public int lift_state = 1;

    public static int lift_dropoff = 3735;
    public static int lift_pickup = 300;
    public static int lift_hook = 650;
    public static int lift_ground = 10;

    private int target;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public Lift(@NonNull HardwareMap hwMap){
        this.liftMotorR = hwMap.get(DcMotorEx.class, "rightLift");
        this.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.liftMotorL = hwMap.get(DcMotorEx.class, "leftLift");
        this.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        this.lift_controller = new PIDController(lift_kP, lift_kI, lift_kD);



    }



    public void manualLift(double input){
        liftMotorL.setPower(input);
        liftMotorR.setPower(input);
        dashboardTelemetry. addData("right pos", liftMotorR.getCurrentPosition());
        dashboardTelemetry. addData("left pos", liftMotorL.getCurrentPosition());
        dashboardTelemetry.update();
    }
    public void liftMove(double power){
        power = power + lift_kH;
        //power = PID_TEst.limiter(power, 1.0);


        liftMotorL.setPower(power);
        liftMotorR.setPower(power);
        dashboardTelemetry. addData("right pos", liftMotorR.getCurrentPosition());
        dashboardTelemetry. addData("left pos", liftMotorL.getCurrentPosition());
        dashboardTelemetry.update();
    }
    public void lift_calc(int target){
        double output = 0.0;
        double shoulder_pos = this.liftMotorR.getCurrentPosition();
        output = lift_controller.calculate(shoulder_pos, target);
        output = PID_TEst.limiter(output, 1.0);


        output = output + lift_kH;

        this.liftMotorR.setPower(output);
        this.liftMotorL.setPower(output);
    }


    public void update(){
        lift_controller.setPID(lift_kP, lift_kI, lift_kD);
        double output = lift_controller.calculate(liftMotorL.getCurrentPosition(), target);
        output = output + lift_kH;
        liftMotorL.setPower(output);
        liftMotorR.setPower(output);
    }
    public double getPosition(){
        return liftMotorL.getCurrentPosition();
    }

    public double getPower(){
        return liftMotorL.getPower();
    }

    public void setLiftDropoff(){
        target = lift_dropoff;
    }
    public void setLiftPickUp(){target = lift_pickup;}
    public void setLiftHook(){target = lift_hook;}
    public void setliftGround(){
        target = lift_ground;
    }
}
