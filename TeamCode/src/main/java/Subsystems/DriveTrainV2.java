package Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class DriveTrainV2 {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private GoBildaPinpointDriver pinpoint;
    private double angle;
    private boolean train_pos;
    private boolean train_on;
    public double fine_move = 1;
    private static double lStickX = 0.0;
    private static double lStickY = 0.0;
    private static double rStickX = 0.0;
    private static double rStickY = 0.0;
    private static double turningSpeedMultiplier = 1;
    private static double robotSpeedMultiplier = 2;
    private static final double TWO_PI = 2 * Math.PI;
    public static double rot_target = 0;
    public static double rot = 0;
    public boolean run_rotational = true;
    public double update_str_target = 0;

    private static PIDController rotational_controller;
    public static double rot_kP = 0.0;
    public static double rot_kI = 0.0;
    public static double rot_kD = 0.0;

    private static PIDController strafe_controller;
    public static double str_kP = 0.0;
    public static double str_kI = 0.0;
    public static double str_kD = 0.0;
    public static double str_pow = 0;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DriveTrainV2 (@NonNull HardwareMap hwMap){

        this.leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        this.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        this.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotational_controller = new PIDController(rot_kP, rot_kI, rot_kD);

        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.recalibrateIMU();
        pinpoint.setOffsets(-4,0);
    }

    public void getController(double a, double b, double c, double d){
        lStickX = a;
        rStickX = b;
        lStickY = c;
        rStickY = d;
        dashboardTelemetry.addData("left Y",lStickY);
        dashboardTelemetry.addData("left X",lStickX);
        dashboardTelemetry.addData("right Y",rStickY);
        dashboardTelemetry.addData("right X",rStickX);
        //dashboardTelemetry.update();
    }

    public void reset_odo(boolean reset){
        if (reset){pinpoint.resetPosAndIMU();}
    }
    public void Drive(double lStickX, double lStickY, double rStickX){

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double y = lStickY; // Remember, this is reversed!
        double x = lStickX; // this is strafing
        double rx = rStickX;

        rot = Rotational_PID();

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = ((y + x + rx) / denominator) - rot + str_pow;
        double leftRearPower = ((y - x + rx) / denominator) - rot - str_pow;
        double rightFrontPower = ((y - x - rx) / denominator) + rot -str_pow;
        double rightRearPower = ((y + x - rx) / denominator) + rot +str_pow;

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftRearPower);
        rightBack.setPower(rightRearPower);
    }


    public void DriveCentric(double lStickX, double lStickY, double rStickX){

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double y = -lStickY; // Remember, this is reversed!
        double x = lStickX; // this is strafing
        double rx = rStickX;
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        double robotAngle = pinpoint.getHeading();

        double theta = Math.atan2(y, x);
        double r = Math.hypot(y, x);

        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        Drive(newRight, newForward, rx);

    }
    public void finecontrols(boolean move){
        if(move && !train_pos){
            train_on = !train_on;
            if (train_on) {
                fine_move = 0.5;
            } else {
                fine_move = 1;
            }
        }
        train_pos = move;
    }

    public double Rotational_PID(){
        rotational_controller.setPID(rot_kP, rot_kI, rot_kD);
        double pow = 0;
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        pow = rotational_controller.calculate(pinpoint.getHeading(), pinpoint.getHeading()+smallestAngleDifference());

        if (rotational_controller.atSetPoint()){
            pow = 0;
            run_rotational = false;
        };
        return pow;
    }
    public void update_run_rotational(){
        if (update_str_target != rot_target){
            update_str_target = rot_target;
            run_rotational = true;
        }
    }

    public  double normalizeToTwoPi(double theta) {

        double normalized = theta % TWO_PI;
        if (normalized < 0) {
            normalized += TWO_PI;
        }

        return normalized;
    }
    public  double smallestAngleDifference() {
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        // Normalize both angles to [0, 2π)
        double current = normalizeToTwoPi(pinpoint.getHeading());

        // Compute the difference and normalize to (-π, π]
        double diff = rot_target - current;
        if (diff > Math.PI) {
            diff -= TWO_PI;
        } else if (diff <= -Math.PI) {
            diff += TWO_PI;
        }
        return diff;

    }


    public void strafePid(double str_target){
        strafe_controller.setPID(str_kP, str_kI, str_kD);
        pinpoint.update();
        str_pow= strafe_controller.calculate(pinpoint.getEncoderX(), str_target);

    }
    public void manualMove(double power){
        leftBack.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    public void train_set_heading_basket(){
        rot_target = 10;
    }


}
