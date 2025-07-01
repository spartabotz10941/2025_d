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
@TeleOp(name = "Subsystem_Test_Arm_V2")
//this line allows you to modify variables inside the dashboard
@Config
public class    Subsystem_Test_ArmV2 extends OpMode {
    private Supersystems supersystems;
    private ElapsedTime runtime = new ElapsedTime();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        supersystems = new Supersystems(hardwareMap);
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
        //if (gamepad1.a){supersystems.armBackwards();}
        //if(gamepad1.b){supersystems.armForward();}
        //if(gamepad1.back){supersystems.armStarting();}
        supersystems.update();


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
