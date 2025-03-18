package Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Hand {
    public Servo wristServo;
    public Servo fingerServo;
    private boolean finger_pos;
    private boolean finger_on;
    private boolean wrist_pos;
    private boolean wrist_on;
    public static double wristDropoffPosition = 1;
    public static double wristPickupPosition = 0;
    public static double wristSpecimenPosition = 0.4;
    public static double wristMiddlePosition = 0.5;
    public static double wristHookPosition = 0.4;
    public static double handOpen = 0;
    public static double handClose = 1;
    private int wrist_state  = 0;



    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public Hand(@NonNull HardwareMap hwMap){
        this.fingerServo = hwMap.get(Servo.class, "fingerServo");
        this.wristServo = hwMap.get(Servo.class, "wristServo");

    }
    public void update(){

    }

    public void moveHand(boolean wrist, boolean finger){
        moveFingers(finger);
        moveWrist1(wrist);
    }
    private void moveFingers(boolean move){
        if(move && !finger_pos){
            finger_on = !finger_on;
            if (finger_on) {
                fingerServo.setPosition(0);
                } else {
                fingerServo.setPosition(1);
                }
            }
        finger_pos = move;
    }
    private void moveWrist1(boolean move){
        if(move && !wrist_pos){
            wrist_on = !wrist_on;
            if (wrist_on) {
                wristServo.setPosition(0);
            } else {
                wristServo.setPosition(1);
            }
        }
        wrist_pos = move;
    }
    private void moveWrist(boolean move){
        if (move){
            wrist_state = wrist_state + 1;
            if (wrist_state == 3){wrist_state = 0;}
        }
        switch (wrist_state){
            case 0:
                wristServo.setPosition(wristDropoffPosition);
            case 1:
                wristServo.setPosition(wristMiddlePosition);
            case 2:
                wristServo.setPosition(wristPickupPosition);
        }
    }
    public double getPosition(){
        return wristServo.getPosition();
    }

    public void setWristDropoff (){
        this.wristServo.setPosition(wristDropoffPosition);
    }
    public  void setWristPickUp(){
        this.wristServo.setPosition(wristPickupPosition);
    }
    public  void setWristSpecimenPickUp(){
        this.wristServo.setPosition(wristSpecimenPosition);
    }
    public void setWristMiddle(){
        this.wristServo.setPosition(wristMiddlePosition);
    }
    public void setWristHook(){
        this.wristServo.setPosition(wristHookPosition);
    }

    public void setHandOpen(){
        this.fingerServo.setPosition(handOpen);
    }
    public void setHandClose(){
        this.fingerServo.setPosition(handClose);
    }

}
