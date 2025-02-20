package Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hand {
    public Servo wristServo;
    public Servo fingerServo;
    private boolean finger_pos;
    private boolean finger_on;
    private boolean wrist_pos;
    private boolean wrist_on;



    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public void init(@NonNull HardwareMap hwMap){
        this.fingerServo = hwMap.get(Servo.class, "fingerServo");
        this.wristServo = hwMap.get(Servo.class, "wristServo");

    }

    public void moveHand(boolean wrist, boolean finger){
        moveFingers(finger);
        moveWrist(wrist);
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
    private void moveWrist(boolean move){
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


}
