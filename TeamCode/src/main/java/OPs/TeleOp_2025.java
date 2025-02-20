package OPs;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.Arm;
import Subsystems.DriveTrain;
import Subsystems.Hand;
import Subsystems.Lift;

@TeleOp (name = "TeleOp 2025")
@Config

public class TeleOp_2025  extends OpMode{
    DriveTrain train = new DriveTrain();
    Lift lift = new Lift();
    Arm arm = new Arm();
    Hand hand = new Hand();
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
        hand.init(hardwareMap);
    }

    @Override
    public void init_loop(){

    }

    @Override
    public void loop(){
        if (gamepad2.left_stick_button) reset_is_done = !reset_is_done;
        if (!reset_is_done){
            //arm.shoulderReset();
            //lift.liftReset();
        }

        gamePadInput();
        train.getController(g1LeftStickX,g1RightStickX,g1LeftStickY, g1RightStickY);
        train.Drive();
        hand.moveHand(gamepad2.a, gamepad2.b);
        //lift.Ascend(- gamepad2.left_stick_y);
        lift.liftMove(-gamepad2.left_stick_y);
        lift.hook_bar(gamepad2.dpad_down, gamepad2.dpad_up);
        //arm.moveShoulder(gamepad2.right_trigger, gamepad2.left_trigger);

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
