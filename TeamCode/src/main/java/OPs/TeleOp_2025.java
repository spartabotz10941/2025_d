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

@TeleOp (name = "TeleOp 2025")
@Config

public class TeleOp_2025  extends OpMode{
    DriveTrain train = new DriveTrain();
    Lift lift = new Lift();
    Arm arm = new Arm();
    public double g1LeftStickY = 0.0;
    public double g1LeftStickX = 0.0;

    public double g1RightStickY = 0.0;
    public double g1RightStickX = 0.0;
    private boolean reset_is_done = false;

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
    }

    @Override
    public void loop(){
        if (!reset_is_done){
            arm.shoulderReset();
            lift.liftReset();
            if (gamepad2.left_stick_button) reset_is_done = false;
        }

        gamePadInput();
        train.getController(g1LeftStickX,g1RightStickX,g1LeftStickY, g1RightStickY);
        train.Drive();
        lift.Ascend(- gamepad2.left_stick_y);
        arm.moveShoulder(gamepad2.right_trigger, gamepad2.left_trigger);

        dashboardTelemetry.addData("shoulder andgle", arm.shoulderAngle());
        dashboardTelemetry.update();

    }


    private void gamePadInput(){
        g1LeftStickY =gamepad1.left_stick_y *-1;
        g1LeftStickX =gamepad1.left_stick_x;

        g1RightStickY =gamepad1.right_stick_y *-1;
        g1RightStickX =gamepad1.right_stick_x;
    }
}
