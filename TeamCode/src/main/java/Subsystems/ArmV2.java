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
    public static double pulleykP = 0.0185;
    public static double pulleykI = 0;
    public static double pulleykD = 0;
    public static double gearkP = 0.0185;
    public static double gearkI = 0;
    public static double gearkD = 0;
    public static int geartargetPosition = 0;
    public static double gearoutput;
    public static double gearPosition;
    public static int pulleytargetPosition = 0;
    public static double pulleyoutput;
    public static double pulleyPosition;

    public static int pulleyForwardPosition = 0;
    public static int pulleyBackPosition  = -1920;
    public static int gearForwardPosition = 1220;
    public static int gearBackPosition = 0;

    public ArmV2(@NonNull HardwareMap hwMap){
        pulleymotor = hwMap.get(DcMotorEx.class,"pulleymotor");
        pulleymotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pulleymotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pulleymotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulleypid = new PIDController(pulleykP, pulleykI, pulleykD);

        gearmotor = hwMap.get(DcMotorEx.class,"gearmotor");
        gearmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gearmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gearmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gearmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        gearpid = new PIDController(gearkP, gearkI, gearkD);

        geartargetPosition = 0;
        pulleytargetPosition = 0;
    }
    public void pulleyupdate(){
        pulleypid.setPID(pulleykP, pulleykI, pulleykD);
        pulleyoutput = pulleypid.calculate(pulleymotor.getCurrentPosition(), pulleytargetPosition);

        pulleyPosition = pulleymotor.getCurrentPosition();

        pulleymotor.setPower(pulleyoutput);
    }

    public int getPulleyPosition(){
        return pulleymotor.getCurrentPosition();
    }

    public void gearupdate(){
        gearpid.setPID(gearkP, gearkI, gearkD);
        gearoutput = gearpid.calculate(gearmotor.getCurrentPosition(), geartargetPosition);

        gearPosition = gearmotor.getCurrentPosition();

        gearmotor.setPower(gearoutput);
    }

    public int getGearPosition(){
        return gearmotor.getCurrentPosition();
    }
    public double getGearPower(){return gearmotor.getPower();}


    public void update(){
        gearupdate();
        pulleyupdate();
        gearPosition = gearmotor.getCurrentPosition();
        pulleyPosition = pulleymotor.getCurrentPosition();
    }

    public void pulleymanualMove(double input1){
        pulleymotor.setPower(input1);
    }
    public void gearSetLimelightTarget (int input){
        geartargetPosition = gearmotor.getCurrentPosition() +input;
    }
    public void gearmanualMove(double input2){
        gearmotor.setPower(input2);

    }

    public boolean isAtState(){
        if(pulleypid.atSetPoint() && gearpid.atSetPoint()){return true;
        }else {return false;}
    }

    public void setPulleyPositionBackward(){pulleytargetPosition = pulleyBackPosition;}
    public void setPulleyPositionForward(){pulleytargetPosition = pulleyForwardPosition;}

    public void setGearPositionForward(){geartargetPosition = gearForwardPosition;}
    public void setGearPositionBackward(){geartargetPosition = gearBackPosition;}
}
