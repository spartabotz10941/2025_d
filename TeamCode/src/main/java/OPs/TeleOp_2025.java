package OPs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.Arm;
import Subsystems.DriveTrain;
import Subsystems.Lift;

@TeleOp
@Config

public class TeleOp_2025  extends OpMode{
    DriveTrain train = new DriveTrain();
    Lift lift = new Lift();
    Arm arm = new Arm();
    public double g1LeftStickY = 0.0;
    public double g1LeftStickX = 0.0;

    public double g1RightStickY = 0.0;
    public double g1RightStickX = 0.0;

    public static double shoulderTarget = 0;

    public static double elbowTarget = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();





    @Override
    public void init(){
        train.init(hardwareMap);
        lift.init(hardwareMap);
        arm.init(hardwareMap);
    }

    @Override
    public void init_loop(){
        arm.shoulderReset();
        arm.elbowReset();
    }

    @Override
    public void loop(){
        gamePadInput();
        train.getController(g1LeftStickX,g1RightStickX,g1LeftStickY, g1RightStickY);
        train.Drive();
        lift.Ascend(gamepad2.right_trigger, gamepad2.left_trigger);
        //arm.mooveShoulder(gamepad2.right_bumper, gamepad2.left_bumper);
        //arm.mooveArm(gamepad2.a, gamepad2.b);
        //arm.shoulder_calc(shoulderTarget);
        //arm.elbow_calc(elbowTarget);

        dashboardTelemetry.addData("shoulder andgle", arm.shoulderAngle());
        dashboardTelemetry.addData("ebow andgle", arm.elbowAngle());
        dashboardTelemetry.update();

    }


    private void gamePadInput(){
        g1LeftStickY =gamepad1.left_stick_y *-1;
        g1LeftStickX =gamepad1.left_stick_x;

        g1RightStickY =gamepad1.right_stick_y *-1;
        g1RightStickX =gamepad1.right_stick_x;
    }
}
