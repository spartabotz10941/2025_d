package Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


//this line identifies if this is an autonomous or teleop program
@TeleOp(name = "Subsystem_Test_Hand")
//this line allows you to modify variables inside the dashboard
@Config
public class    Subsystem_Test_Hand extends OpMode {
    public static Hand hand;
    public static double input_lr;
    public static double input_ud;
    public static double input_rollers;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double i =0;
    public static boolean position;

    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    GamepadEx g1;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        hand = new Hand(hardwareMap);
        hand.limelightStart(hand.limelightRed);
        // Tell the driver that initialization is complete.
        dashboardTelemetry.addData("Status", "Initialized");
        dashboardTelemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {
       hand.updateLimelightData();

       if (position){
           i = hand.turnToAngle();
       }


       dashboardTelemetry.addData("angle", hand.angle);
       dashboardTelemetry.addData("tx", hand.tx);
       dashboardTelemetry.addData("ty", hand.ty);
       dashboardTelemetry.addData("temperature", hand.temperature);
       dashboardTelemetry.addData("servo",  i);

       dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        hand.stopLimelight();
        dashboardTelemetry.update();
    }
}
