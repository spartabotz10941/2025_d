package Subsystems;

import static java.lang.Thread.sleep;

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
public class Arm {
    //motors
    private static DcMotorEx shoulderMotor;

    public static double motor_power = 0.2;

    // buttons
    TouchSensor shoulderTouch;
    private boolean shoulderIsReset = false;

    //shoulder PID
    public static PIDController shoulder_controller;
    public static double shoulder_kP = 0.004;
    public static double shoulder_kI = 0.01;
    public static double shoulder_kD = 0;
    public static double shoulder_hold = 0.15;
    public static double shoulder_ticks_per_radians = 1000;

    private double shoulder_target;
    public static double shoulder_max_position = 100;
    public static double shoulder_min_position = -1300;
    public static double shoulder_target_change = 20;
    public static double shoulder_start_angle = 2.2483675;


    //arm movement
    public static final double arm1Length = 242; // length of arm1 in mm
    public static final double arm2Length = 320; // length of arm2 in mm
    public static final double clawLength = 167; // length of claw in mm
    public static double desiredHeight;
    public static double desiredLength;
    public static double arm2Trig = Math.sqrt(Math.pow(arm2Length,2) + Math.pow(clawLength,2));
    public static double maxExtention = arm1Length + arm2Length - 170; // max Extension in mm
    public static double minExtention = 0; // min Extension in mm
    public static double theta1 = 0;
    public static double theta2 = 0;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.shoulderMotor = hwMap.get(DcMotorEx.class, "shoulderMotor" );
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.shoulder_controller = new PIDController (shoulder_kP, shoulder_kI, shoulder_kD);

        shoulderTouch = hwMap.get(TouchSensor.class,"shoulderReset" );
        shoulder_target = shoulderMotor.getCurrentPosition();

    }

    public void shoulderReset(){
        if (!shoulderTouch.isPressed() & !shoulderIsReset){
            shoulderMotor.setPower(0.2);
            dashboardTelemetry.addData("shoulder is reset", shoulderIsReset);
            if (shoulderTouch.isPressed()) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                shoulderIsReset = true;
                dashboardTelemetry.addData("shoulder is reset", shoulderIsReset);
            }
        }
        dashboardTelemetry.update();
    }
    public void moveShoulder(double up, double down){
        double direction = up - down;
        this.shoulder_target = shoulder_target + direction * shoulder_target_change;
        if (shoulder_target< shoulder_min_position){
            shoulder_target = shoulder_min_position;
        }else if (shoulder_target > shoulder_max_position){
            shoulder_target = shoulder_max_position;
        }
        dashboardTelemetry. addData("shoulder target", shoulder_target);
        dashboardTelemetry.update();

        shoulder_calc(shoulder_target);

    }

    public static void shoulder_calc (double shoulder_target){
        double output = 0.0;
        double shoulder_pos = shoulderMotor.getCurrentPosition();
        if (Math.abs(shoulder_target-shoulder_pos) > 5) {
            output = shoulder_controller.calculate(shoulder_pos, shoulder_target);
            output = PID_TEst.limiter(output, 1.0);

        }
        double shoulderAngle = shoulder_start_angle - (shoulderMotor.getCurrentPosition()/shoulder_ticks_per_radians);
        shoulderAngle = shoulderAngle + shoulder_start_angle;
        shoulderMotor.setPower(output);
    }

    public double shoulderAngle(){
        double shoulderAngle = shoulder_start_angle + (shoulderMotor.getCurrentPosition()/shoulder_ticks_per_radians);
        dashboardTelemetry.addData("shoulder position", shoulderMotor.getCurrentPosition());
        dashboardTelemetry.addData("shoulder angle", shoulderAngle);
        return shoulderAngle;
    }


    /*
    public void elbowReset(){
        if (!elbowTouch.isPressed() & !elbowIsReset & shoulderIsReset){
            elbowMotor.setPower(0.5);
            dashboardTelemetry.addData("elbow is reset", elbowIsReset);
        }else if (elbowTouch.isPressed()){
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            elbowMotor.setPower(0);
            elbowIsReset = !elbowIsReset;
            dashboardTelemetry.addData("elbow is reset", elbowIsReset);
        }
        dashboardTelemetry.update();
    }

    public void mooveArm (boolean up, boolean down){

        dashboardTelemetry.addData("befor up values", up);

        if (up){
            dashboardTelemetry.addData("up values",up);
            dashboardTelemetry.update();
            elbowMotor.setPower(motor_power);
        }
        else if (down) {
            elbowMotor.setPower(-motor_power);
            dashboardTelemetry.addData("down values", down);
            dashboardTelemetry.update();
        }
        else {
            dashboardTelemetry.addData("up else", up);
            elbowMotor.setPower(0.0);

        }
    }

    //pid controller output for shoulder motor


    public static void elbow_calc(double elbow_target){
        double output = 0.0;
        double shoulder_pos = elbowMotor.getCurrentPosition();
        if (Math.abs(elbow_target-shoulder_pos) > 5) {
            output = elbow_controller.calculate(shoulder_pos, elbow_target);
            output = PID_TEst.limiter(output, 1.0);

        }
        //elbowMotor.setPower(output + (elbow_hold * Math.cos(elbowAngle())));
    }

    public  double elbowAngle(){
        double elbowAngle = (Math.PI / 2)-(elbowMotor.getCurrentPosition()/elbow_ticks_per_radians);
        //dashboardTelemetry.addData("elbow position", elbowMotor.getCurrentPosition());
        //dashboardTelemetry.addData("elbow angle", elbowAngle);
        return elbowAngle;
    }



    public static void controllerToPosition(double l, double r){

        desiredLength = desiredLength + (l * lengthSpeed);
        if (desiredLength > maxExtention){
            desiredLength = maxExtention;
        } else if (desiredLength<minExtention) {
            desiredLength=minExtention;
        }

        desiredHeight = desiredHeight + r * heightSpeed;
        if (desiredHeight > maxExtention){
            desiredHeight = maxExtention;
        } else if (desiredHeight<minExtention) {
            desiredHeight=minExtention;
        }

        armMath();
        mathTest();

    }

    // arm math angle
    public static void armMath() {

        double p = Math.sqrt(Math.pow(desiredLength,2) + Math.pow(desiredHeight,2));

        theta1 = Math.PI - Math.atan(desiredHeight / desiredLength) - Math.acos( ( Math.pow(arm2Trig, 2) - Math.pow(arm1Length,2) - Math.pow(q,2)) / (-2 * arm1Length * q) );
        theta2 = Math.acos( (Math.pow(q,2) - Math.pow(arm1Length, 2) - Math.pow(arm2Length, 2)) / (-2 * arm1Length * arm2Length));


        theta2 = Math.PI + Math.acos( (Math.pow(desiredLength, 2) + Math.pow(desiredHeight, 2) - Math.pow(arm1Length, 2) - Math.pow(arm2Trig, 2)) / (2 * arm1Length * arm2Trig) );
        theta1 = (Math.PI / 2) - Math.atan(desiredHeight / desiredLength) + Math.atan( (arm2Trig * Math.sin(theta2)) / (arm1Length + (arm2Trig * Math.cos(arm2Trig))));


        theta1 = (Math.PI / 2) - Math.atan(desiredHeight / desiredLength) - Math.acos( ( Math.pow(arm2Trig, 2) - Math.pow(arm1Length, 2) - Math.pow(p, 2)) / ( -2 * p * arm1Length) );
        theta2 = Math.acos( (Math.pow(p, 2) - Math.pow(arm1Length, 2) - Math.pow(arm2Trig, 2)) / (-2 * arm1Length * arm2Trig) );


        //shoulder_calc (shoulderMotor.getCurrentPosition(), theta1 * shoulder_ticks_per_radians * -1);

        //elbow_calc(elbowMotor.getCurrentPosition(), theta2 * elbow_ticks_per_radians);

    }

    public static void mathTest() {
        thetaTest = Math.acos((Math.pow(d, 2) - Math.pow(b, 2) - Math.pow(c, 2)) / (-2 * b * c));
    }
    */

}