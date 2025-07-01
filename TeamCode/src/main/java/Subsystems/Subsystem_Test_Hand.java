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
    public static double angle;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public static double rollers =0;
    public static double udservoinput = 0.5;
    public static double lrservoinput=0.5;
        public static boolean position = true;

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
        hand.limelightSwitch(hand.limelightRed);
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

        /*
        hand.updateLimelightData();

       if (position){
           i = hand.turnToAngle();
       }*/
        hand.updateLimelightData();
        hand.rollers(rollers);
        hand.moveudservo(udservoinput);
        if (gamepad1.a && position){
            angle =hand.turnToAngle();
            position = false;
        }
        if (gamepad1.b){position = true;}
        if (gamepad1.x){hand.stopLimelight();}


        dashboardTelemetry.addData("angle", hand.angle);
       dashboardTelemetry.addData("tx", hand.tx);
       dashboardTelemetry.addData("ty", hand.ty);
       dashboardTelemetry.addData("temperature", hand.temperature);
       dashboardTelemetry.addData("angle", angle);


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
