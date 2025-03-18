package Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;


//this line identifies if this is an autonomous or teleop program
@Autonomous(name = "Subsystem_Test_Lift")
//this line allows you to modify variables inside the dashboard
@Config
public class Subsystem_Test_LIft extends OpMode {
    public static ArmV2 arm2;

    private MyLimeLight myLimeLight;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motor0;

    public static Lift lift ;
    public static Hand hand ;
    public static int i =0;
    public static int position = 0;

    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    GamepadEx g1;
    Supersystems supersystem;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*
        this.motor0 = hardwareMap.get(DcMotorEx.class,"shoulderMotor");
        this.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hand.init(hardwareMap);
         */
        lift = new Lift(hardwareMap);
        supersystem = new Supersystems(hardwareMap);
        myLimeLight = new MyLimeLight(hardwareMap);
        myLimeLight.start(myLimeLight.red);

        g1 = new GamepadEx(gamepad1);
        arm2 = new ArmV2(hardwareMap);
        // Tell the driver that initialization is complete.
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
        //motor0.setPower(power);
        //dashboardTelemetry.addData("wrist position", hand.wristServo.getPosition());
        //dashboardTelemetry.addData("motor0 Position", motor0.getCurrentPosition());
        //arm2.setPosition(i);
        //arm2.update();
        //lift.update(i);
        //supersystem.basePosition();
        //List<Double> coordinates = myLimeLight.getCoordinatesList();
        myLimeLight.update();


        dashboardTelemetry.addData("SAMPLE x", myLimeLight.getTx());
        dashboardTelemetry.addData("SAMPLE y", myLimeLight.getTy());
        dashboardTelemetry.addData("encoder" , arm2.getPosition());
        dashboardTelemetry.addData("power" , arm2.getPower());
        dashboardTelemetry.addData("target", i);
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
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
