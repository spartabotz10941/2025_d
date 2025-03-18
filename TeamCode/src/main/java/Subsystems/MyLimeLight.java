package Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class MyLimeLight {
    private final Limelight3A limelight;
    private double xLoc;
    private double yLoc;
    private double tx;
    private double ty;
    private double angle;

    public int red = 0;
    public int blue = 1;
    public int yellow = 2;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public MyLimeLight(@NonNull HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void start(int input){
        limelight.pipelineSwitch(input);
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }

    public boolean update(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            //ta = result.getTa();
            dashboardTelemetry.addData("tx", tx);
            dashboardTelemetry.addData("ty", ty);
            //dashboardTelemetry.update();

            // Getting numbers from Python
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                xLoc = pythonOutputs[1];
                yLoc = pythonOutputs[2];
                //angle = pythonOutputs[3];
            }
            return true;
        }
        return false;
    }

    public double getxLoc(){
        return xLoc;
    }

    public double getyLoc(){
        return yLoc;
    }

    public double getAngle(){
        return angle;
    }

    public double getTx(){
        return tx;
    }
    public double getTy(){
        return ty;
    }

    public List<Double> getCoordinatesList() {
        List<Double> coordinates = new ArrayList<>();
        coordinates.add(getTx());
        coordinates.add(getTy() );
        return coordinates;
    }



}