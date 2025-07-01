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


//this line identifies if this is an autonomous or teleop program
@TeleOp(name = "Subsystem_Test")
//this line allows you to modify variables inside the dashboard
@Config
public class    Subsystem_Test extends OpMode {
    public static ArmV2 arm2;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motor0;

    public static Lift lift ;
    public static Hand hand ;
    public static DriveTrainV2 drive;
    public static int i =0;
    public static int position = 0;

    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    GamepadEx g1;

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
        lift.init(hardwareMap);

         */
        g1 = new GamepadEx(gamepad1);
        arm2 = new ArmV2(hardwareMap);
        lift = new Lift(hardwareMap);
        hand = new Hand (hardwareMap);
        drive = new DriveTrainV2(hardwareMap);
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

        dashboardTelemetry.addData("heading", drive.getYaw());
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
