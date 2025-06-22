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
@TeleOp(name = "Subsystem_Test_Arm_V2")
//this line allows you to modify variables inside the dashboard
@Config
public class    Subsystem_Test_ArmV2 extends OpMode {
    public static ArmV2 arm;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
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
        arm = new ArmV2(hardwareMap);
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
        arm.update();
        // -1956

        dashboardTelemetry.addData("Status", "Run Time: " + runtime.toString());
        dashboardTelemetry.addData("gear PID", ArmV2.gearoutput);
        dashboardTelemetry.addData("gear target", ArmV2.geartargetPosition);
        dashboardTelemetry.addData("gear current Position", ArmV2.gearPosition);

        dashboardTelemetry.addData("pulley PID", ArmV2.pulleyoutput);
        dashboardTelemetry.addData("pulley target", ArmV2.pulleytargetPosition);
        dashboardTelemetry.addData("pulley current Position", ArmV2.pulleyPosition);

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
