package Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import Subsystems.Arm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


//this line identifies if this is an autonomous or teleop program
@TeleOp(name = "Math_Test")
//this line allows you to modify variables inside the dashboard
@Config
public class mathTest extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        Arm arm = new Arm();
        arm.init(hardwareMap);

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
        Arm.controllerToPosition (gamepad2.left_stick_y, gamepad2.right_stick_y);

        dashboardTelemetry.addData("gamepad left", gamepad2.left_stick_y);
        dashboardTelemetry.addData("gamepad right", gamepad2.right_stick_y);
        dashboardTelemetry.addData("theta 1", Arm.theta1);
        dashboardTelemetry.addData("theta2", Arm.theta2);
        dashboardTelemetry.addData("desired height", Arm.desiredHeight);
        dashboardTelemetry.addData("desired length", Arm.desiredLength);
        dashboardTelemetry.addData("armTrig", Arm.arm2Trig);
        dashboardTelemetry.addData("max extention", Arm.maxExtention);
        dashboardTelemetry.addData("min extention", Arm.minExtention);
        dashboardTelemetry.addData("arm1Length", Arm.arm1Length);
        dashboardTelemetry.addData("arm2Length", Arm.arm2Length);
        dashboardTelemetry.addData("clawLength", Arm.clawLength);
        /*
        dashboardTelemetry.addData("Input b =", Arm.b);
        dashboardTelemetry.addData("Input c =", Arm.c);
        dashboardTelemetry.addData("Input d =", Arm.d);
        dashboardTelemetry.addData("Output ThetaTest:", Arm.thetaTest);
         */

        dashboardTelemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
