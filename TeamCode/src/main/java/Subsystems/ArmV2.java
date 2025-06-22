package Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ArmV2 {
    DcMotorEx pulleymotor;
    DcMotorEx gearmotor;
    PIDController pulleypid;
    PIDController gearpid;
    public static double pulleykP = 0.0;
    public static double pulleykI = 0;
    public static double pulleykD = 0;
    public static double gearkP = 0.0;
    public static double gearkI = 0;
    public static double gearkD = 0;
    public static int posDown = -823;
    public static int geartargetPosition;
    public static double gearoutput;
    public static double gearPosition;
    public static int pulleytargetPosition;
    public static double pulleyoutput;
    public static double pulleyPosition;

    public static int pulleyForwardPosition = 0;
    public static int pulleyBackPosition  = -1940;

    public ArmV2(@NonNull HardwareMap hwMap){
        pulleymotor = hwMap.get(DcMotorEx.class,"pulleymotor");
        pulleymotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleymotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //pulleymotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulleypid = new PIDController(pulleykP, pulleykI, pulleykD);

        gearmotor = hwMap.get(DcMotorEx.class,"gearmotor");
        gearmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gearmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gearmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gearmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gearpid = new PIDController(gearkP, gearkI, gearkD);
    }
    public void pulleyupdate(){
        pulleypid.setPID(pulleykP, pulleykI, pulleykD);
        pulleyoutput = pulleypid.calculate(pulleymotor.getCurrentPosition(), pulleytargetPosition);

        pulleyPosition = pulleymotor.getCurrentPosition();

        pulleymotor.setPower(pulleyoutput);
    }

    public void gearupdate(){
        gearpid.setPID(gearkP, gearkI, gearkD);
        gearoutput = gearpid.calculate(gearmotor.getCurrentPosition(), geartargetPosition);

        gearPosition = gearmotor.getCurrentPosition();

        pulleymotor.setPower(gearoutput);
    }

    public int getGearPosition(){
        return gearmotor.getCurrentPosition();
    }
    public double getGearPower(){
        return gearmotor.getPower();
    }


    public void update(){
        gearupdate();
        pulleyupdate();
        gearPosition = gearmotor.getCurrentPosition();
        pulleyPosition = pulleymotor.getCurrentPosition();
    }

    public void pulleymanualMove(double input){
        pulleymotor.setPower(input);
    }
}
