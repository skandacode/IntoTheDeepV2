package subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

import subsystems.Intake;
@Config
@TeleOp
public class IntakeTestingClass extends LinearOpMode {
    Intake intake;

    public static int extendoPos=0;
    public static double flipPos=0.95;
    public static double intakeP=0;
    public static boolean cover=true;
    public static boolean sweeper=true;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            intake.setIntakeFlip(flipPos);
            intake.setIntakePower(intakeP);
            intake.setCover(cover);
            intake.setSweeper(sweeper);
            intake.setTargetPos(extendoPos);
            telemetry.addData("intake extendo pos", intake.getExtendoMotorPosition());
            telemetry.addData("intake limit", intake.isRetracted());
            telemetry.addData("intake analog", intake.getFlipAnalog());
            telemetry.addData("intake raw values", Arrays.toString(intake.getRawSensorValues()));
            telemetry.addData("intake sensor dist", intake.getDistance());

            telemetry.update();
            intake.update();
        }
    }
}
