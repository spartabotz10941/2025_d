package Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import java.util.List;


public class Limelight {
    public Servo wristSide;
    public Servo wristUp;
    public Limelight3A limelight;
    public LLResult result;
    public List<LLResultTypes.ColorResult>  colorTargets;
    public double tx = 0;
    public double ty = 0;
    public List<List<Double>> corners;




    public Limelight(@NonNull HardwareMap hwMap) {
        this.wristSide = hwMap.get(Servo.class, "wristSide");
        this.wristUp = hwMap.get(Servo.class, "wristUp");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

    }


    public void updateLimelightData() {
        result = limelight.getLatestResult();
        if (result != null & result.isValid()) {
            colorTargets = result.getColorResults();
            for (LLResultTypes.ColorResult colorTarget : colorTargets) {
                tx = colorTarget.getTargetXDegrees(); // Where it is (left-right)
                ty = colorTarget.getTargetYDegrees(); // Where it is (up-down)
                corners = colorTarget.getTargetCorners(); // size (0-100)

            }
        }
    }

    public double getXOffset(){
        return tx;
    }
    public double getYOffset(){
        return ty;
    }
    public List<List<Double>> getcorners(){
        return corners;
    }
    public void stopLimelight(){
        limelight.stop();
    }

}
