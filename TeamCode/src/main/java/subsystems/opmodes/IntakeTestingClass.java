package subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.Intake;
@Config
@TeleOp
public class IntakeTestingClass extends LinearOpMode {
    Intake intake;

    public static int extendoPos=0;
    public static double flipPos=0.43;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            intake.setTargetPos(extendoPos);
            intake.setIntakeFlip(flipPos);
            intake.update();
            telemetry.addData("intake extendo pos", intake.getExtendoMotorPosition());
            telemetry.addData("intake limit", intake.isRetracted());
            telemetry.addData("intake analog", intake.getFlipAnalog());
            telemetry.update();
        }
    }
}
