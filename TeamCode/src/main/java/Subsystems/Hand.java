package Subsystems;

import static android.graphics.Color.argb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Hand {
    public Servo lrservo;
    public double lrinput ;
    public Servo udservo;
    public double udinput ;
    public CRServo contservo1;
    public CRServo contservo2;

    public ColorSensor color;
    public DistanceSensor distance;
    public LLResult result;

    public Limelight3A limelight;
    public int limelightRed = 3;
    public int limelightBlue = 5;
    public int limelightYellow = 4;
    public double temperature;
    public double tx;
    public double gear_ticks_per_units = 5.0;
    public double ty;
    public double wheels_ticks_per_unit =5.0;
    public double angle;
    public double[] pythonOutputs;


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public Hand(@NonNull HardwareMap hwMap){
        this.lrservo = hwMap.get(Servo.class, "lrservo");
        movelrservo(0.5);

        this.udservo = hwMap.get(Servo.class, "udservo");
        moveudservo(0.5);
        this.contservo1 = hwMap.get(CRServo.class, "contservo1");
        this.contservo2 = hwMap.get(CRServo.class,"contservo2");
        this.color = hwMap.get(ColorSensor.class, "color");
        this.distance = hwMap.get(DistanceSensor.class, "distance");

        limelight = hwMap.get(Limelight3A.class, "limelight");


    }
    public void limelightSwitch(int input){
        limelight.pipelineSwitch(input);
        limelight.setPollRateHz(2);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        limelight.start();


    }
    public void limelightStart(){
        limelight.start();
    }

    public void moveudservo(double input ){
        //1 is left and 0 is right
        udinput = input;
        udservo.setPosition(udinput);}
    public void movelrservo(double input){
        //1 is behind 0 is forward
        lrinput = input;
        lrservo.setPosition(lrinput);}
    public void rollers(double input){
        contservo1.setPower(-input);
        contservo2.setPower(input);
    }

    public double distance(){
        return distance.getDistance(DistanceUnit.CM);
    }

    public int color(){
        return color.argb();
    }

    public void updateLimelightData() {
        LLStatus status = limelight.getStatus();
        temperature = status.getTemp();

        result = limelight.getLatestResult();
        tx = result.getTx();
        ty = result.getTy();

        pythonOutputs = result.getPythonOutput();

        if (result != null && result.isValid()){

        }

        if (pythonOutputs != null && pythonOutputs.length > 0) {
            angle = pythonOutputs[3];
        }
    }
    public int getDistanceFromTx(){
        return (int) (tx*wheels_ticks_per_unit);
    }

    public int getDistanceFromTy(){
        return (int) (ty*gear_ticks_per_units);
    }

    public double turnToAngle(){
        double a = angle +90;
        if (a > 180){a = a % 180;}

        a = a/180;
        lrservo.setPosition(a);
        return a;
    }

    public void stopLimelight(){
        //limelight.stop();
        limelight.shutdown();
    }
}
