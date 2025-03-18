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

    private DcMotorEx hookMotor;
    TouchSensor liftTouch;
    private boolean lift_is_reset = false;
    private int lift_target = 0;
    public static int lift_target_change = 0;
    public static int lift_max_position = 3700;
    private static PIDController lift_controller;
    public static double lift_kP = 0.01;
    public static double lift_kI = 0.0;
    public static double lift_kD = 0.0;
    public static double lift_kH = 0.0009;

    public int lift_state = 1;

    public static int lift_dropoff = 3350;
    public static int lift_pickup = 10;
    public static int lift_hook = 650;
    public static int lift_lift = 0;

    private int target;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public Lift(@NonNull HardwareMap hwMap){
        this.liftMotorR = hwMap.get(DcMotorEx.class, "liftMotor");
        this.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        this.liftMotorL = hwMap.get(DcMotorEx.class, "liftMotor2");
        this.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.hookMotor = hwMap.get(DcMotorEx.class, "hookMotor");
        this.hookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.hookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.hookMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.hookMotor.setPower(-0.1);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        this.hookMotor.setPower(0);

        this.lift_controller = new PIDController(lift_kP, lift_kI, lift_kD);

        liftTouch = hwMap.get(TouchSensor.class,"liftReset" );

    }

    public void liftReset(){
        if (!liftTouch.isPressed() & !lift_is_reset){
            this.liftMotorR.setPower(-0.2);
            this.liftMotorL.setPower(-0.2);
            dashboardTelemetry.addData("shoulder is reset", lift_is_reset);
            if (liftTouch.isPressed()) {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                this.liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                this.liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.liftMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

                lift_is_reset = true;
                dashboardTelemetry.addData("shoulder is reset", lift_is_reset);
            }
        }
        dashboardTelemetry.update();
    }

    public void Ascend(double direction){
        this.lift_target = (int) (lift_target + (direction * lift_target_change));
        if (lift_target< 0){
            lift_target = 0;
        }else if (lift_target > lift_max_position){
            lift_target = lift_max_position;
        }
        dashboardTelemetry.addData("lift target", lift_target);
        dashboardTelemetry.update();
        lift_calc(lift_target);
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

    public void hook_bar (boolean hook, boolean unhook){
        if (hook){
            hookMotor.setPower(0.1);
        } else if (unhook){
            hookMotor.setPower(-0.1);
        }else {
            hookMotor.setPower(0);
        }

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
    public void setLiftPickUp(){
        target = lift_pickup;

    }
    public void setLiftHook(){
        target = lift_hook;

    }
    public void setliftlift(){
        target = lift_lift;
    }
    public void liftControlSwitch(){
        switch (lift_state){
            case 0:
                setLiftPickUp();

            case 1:
                setLiftDropoff();
        }
    }
}
