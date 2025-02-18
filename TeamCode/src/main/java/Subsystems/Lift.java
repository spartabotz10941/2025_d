package Subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    private DcMotorEx liftMotorR;
    private DcMotorEx liftMotorL;

    private static PIDController lift_controller;
    public static double lift_kP = 0;
    public static double lift_kI = 0;
    public static double lift_kD = 0;
    public static double lift_kH = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.liftMotorR = hwMap.get(DcMotorEx.class, "liftMotor");
        this.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.liftMotorL = hwMap.get(DcMotorEx.class, "liftMotor2");
        this.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void Ascend(double up, double down){
        double direction = 0.0;
        direction = up - down;
        if (direction != 0.0) {
            liftMotorR.setPower(direction);
            liftMotorL.setPower(direction);
        }
        else{
            liftMotorR.setPower(0.0);
            liftMotorL.setPower(0.0);
        }
    }

    public void lift_calc(double shoulder_target){
        double output = 0.0;
        double shoulder_pos = this.liftMotorR.getCurrentPosition();
        if (Math.abs(shoulder_target-shoulder_pos) > 5) {
            output = lift_controller.calculate(shoulder_pos, shoulder_target);
            output = PID_TEst.limiter(output, 1.0);
        }

        output = output + lift_kH;

        this.liftMotorR.setPower(output);
        this.liftMotorL.setPower(output);
    }
}
