package Subsystems;

import androidx.annotation.NonNull;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Arm {
    //motors
    private static DcMotorEx shoulderMotor;
    private static DcMotorEx elbowMotor;
    public static double motor_power = 0.2;

    //shoulder PID
    private static PIDController shoulder_controller;
    public static double shoulder_kP = 0.01;
    public static double shoulder_kI = 0.028;
    public static double shoulder_kD = 0;
    public static double shoulder_ticks_per_radians = 900;

    //elbow PID
    private static PIDController elbow_controller;
    public static double elbow_kP = 0.003;
    public static double elbow_kI = 0.05;
    public static double elbow_kD = 0;
    public static double elbow_ticks_per_radians = 1760;

    //arm movement
    public static final double arm1Length = 242; // length of arm1 in mm
    public static final double arm2Length = 320; // length of arm2 in mm
    public static final double clawLength = 167; // length of claw in mm
    public static double desiredHeight;
    public static double heightSpeed = 0.25;
    public static double desiredLength;
    public static double lengthSpeed = 0.25;
    public static double arm2Trig = Math.sqrt(Math.pow(arm2Length,2) + Math.pow(clawLength,2));
    public static double maxExtention = arm1Length + arm2Length - 170; // max Extension in mm
    public static double minExtention = 0; // min Extension in mm
    public static double theta1 = 0;
    public static double theta2 = 0;

    public static double b;
    public static double c;
    public static double d;
    public static double thetaTest;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.shoulderMotor = hwMap.get(DcMotorEx.class, "shoulderMotor" );
        this.shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.shoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.shoulderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shoulderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.shoulder_controller = new PIDController (shoulder_kP, shoulder_kI, shoulder_kD);

        this.elbowMotor = hwMap.get(DcMotorEx.class, "elbowMotor" );
        this.elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.elbow_controller = new PIDController(elbow_kP, elbow_kI, elbow_kD);


    }
    public void mooveShoulder(boolean upp, boolean downn){
       if (upp){
           shoulderMotor.setPower(motor_power);
        }
       else if (downn) {
           shoulderMotor.setPower(-motor_power);
       }
       else {
           shoulderMotor.setPower(0.0);
       }

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
    public static void shoulder_calc (double shoulder_pos, double shoulder_target){
        double output = 0.0;
        if (Math.abs(shoulder_target-shoulder_pos) > 5) {
            output = shoulder_controller.calculate(shoulder_pos, shoulder_target);
            output = PID_TEst.limiter(output, 1.0);

        }
        shoulderMotor.setPower(output);
    }

    public static void elbow_calc(double shoulder_pos, double shoulder_target){
        double output = 0.0;
        if (Math.abs(shoulder_target-shoulder_pos) > 5) {
            output = elbow_controller.calculate(shoulder_pos, shoulder_target);
            output = PID_TEst.limiter(output, 1.0);

        }
        elbowMotor.setPower(output);
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
        /*
        double q = Math.sqrt(Math.pow(desiredLength,2) + Math.pow(desiredHeight,2));

        theta1 = Math.PI - Math.atan(desiredHeight / desiredLength) - Math.acos( ( Math.pow(arm2Trig, 2) - Math.pow(arm1Length,2) - Math.pow(q,2)) / (-2 * arm1Length * q) );
        theta2 = Math.acos( (Math.pow(q,2) - Math.pow(arm1Length, 2) - Math.pow(arm2Length, 2)) / (-2 * arm1Length * arm2Length));
        */

        theta2 = Math.PI + Math.acos( (Math.pow(desiredLength, 2) + Math.pow(desiredHeight, 2) - Math.pow(arm1Length, 2) - Math.pow(arm2Trig, 2)) / (2 * arm1Length * arm2Trig) );
        theta1 = (Math.PI / 2) - Math.atan(desiredHeight / desiredLength) + Math.atan( (arm2Trig * Math.sin(theta2)) / (arm1Length + (arm2Trig * Math.cos(arm2Trig))));

        //shoulder_calc (shoulderMotor.getCurrentPosition(), theta1 * shoulder_ticks_per_radians * -1);

        //elbow_calc(elbowMotor.getCurrentPosition(), theta2 * elbow_ticks_per_radians);

    }

    public static void mathTest() {
        thetaTest = Math.acos((Math.pow(d, 2) - Math.pow(b, 2) - Math.pow(c, 2)) / (-2 * b * c));
    }


}