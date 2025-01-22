package subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.Outtake;

@Config
@TeleOp
public class OuttakeTestingClass extends LinearOpMode {
    Outtake outtake;
    public static double wristpos = 0.5;
    public static double flippos = 0.5;
    public static double railpos = 0.5;
    public static double clawpos = 0.8;
    public static int outtakeTargetPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            outtake.setWrist(wristpos);
            outtake.setFlip(flippos);
            outtake.setRail(railpos);
            outtake.setClaw(clawpos);
            outtake.setTargetPos(outtakeTargetPos);
            outtake.update();
            telemetry.addData("Outtake Pos", outtake.getCachedPos());
            telemetry.addData("Flip Pos", outtake.getFlipAnalog());
            telemetry.update();
        }
    }
}
