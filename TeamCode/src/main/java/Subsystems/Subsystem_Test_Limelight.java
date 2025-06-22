package Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Subsystem_Test")
//this line allows you to modify variables inside the dashboard

public class    Subsystem_Test_Limelight extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static Limelight lmg;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        lmg = new Limelight(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        lmg.updateLimelightData();
        dashboardTelemetry.addData("X", lmg.getXOffset());
        dashboardTelemetry.addData("Y", lmg.getYOffset());
        dashboardTelemetry.addData("corners", lmg.getcorners());
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        lmg.stopLimelight();
    }
}
