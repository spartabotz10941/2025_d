package Subsystems;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class DriveTrain {
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private GoBildaPinpointDriver pinpoint;
    private double angle;
    private static double lStickX = 0.0;
    private static double lStickY = 0.0;
    private static double rStickX = 0.0;
    private static double rStickY = 0.0;
    private static double turningSpeedMultiplier = 1;
    private static double robotSpeedMultiplier = 2;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init (@NonNull HardwareMap hwMap){

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

        pinpoint = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.resetPosAndIMU();
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
    public void Drive(double lStickX, double lStickY, double rStickX){

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double y = lStickY; // Remember, this is reversed!
        double x = lStickX; // this is strafing
        double rx = rStickX;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

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



}