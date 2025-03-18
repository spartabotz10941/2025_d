package Subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Limelight extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(20);

        limelight.pipelineSwitch(0);
        limelight.pipelineSwitch(1);
        limelight.pipelineSwitch(2);

        /*
         * Starts polling for data.
         */
        limelight.start();

    }}