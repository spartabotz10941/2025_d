package Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


//this line identifies if this is an autonomous or teleop program
@Autonomous(name = "Subsystem_Test")
//this line allows you to modify variables inside the dashboard
@Config
public class Subsystem_Test extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motor0;
    private Servo servo0;
    public static double power =0.0;
    public static int position = 0;

    //this section allows us to access telemetry data from a browser
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.motor0 = hardwareMap.get(DcMotorEx.class,"shoulderMotor");
        this.motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.servo0 = hardwareMap.get(Servo.class, "fingerServo");

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
//        this.motor0.getCurrentPosition();
//
//        this.motor0.setTargetPosition(position);
//        this.motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motor0.setPower(power);

        dashboardTelemetry.addData("motor0 power", motor0.getPower());
        dashboardTelemetry.addData("motor0 Position", motor0.getCurrentPosition());
        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());

        dashboardTelemetry.addData("servo position", servo0.getPosition());
        dashboardTelemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        this.motor0.setPower(0);
        dashboardTelemetry.addData("motor0 power", motor0.getPower());
        dashboardTelemetry.update();
    }
}
