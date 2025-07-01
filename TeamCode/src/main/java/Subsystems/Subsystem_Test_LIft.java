package Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


//this line identifies if this is an autonomous or teleop program
@TeleOp(name = "Subsystem_Test_Lift")
//this line allows you to modify variables inside the dashboard
@Config
public class Subsystem_Test_LIft extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static Lift lift ;
    public static double input = 0;
    public static boolean controller = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        lift = new Lift(hardwareMap);
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
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
        if (controller){
            lift.liftsettarget((int) input);
            lift.update();
        }else {

            lift.manualLift(input);
        }

        dashboardTelemetry.addData("lift position", lift.getPosition());
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        dashboardTelemetry.update();
    }
}
