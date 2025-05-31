import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import subsystems.Intake;

@Config
@TeleOp
public class IntakeCurrentMonitoring extends LinearOpMode {
    Intake intake;
    public static double intakePower=0;

    @Override
    public void runOpMode() throws InterruptedException {
        AutomatedTeleopSample.maxExtend=400;
        AutomatedTeleopSpec.maxExtend=400;

        intake=new Intake(hardwareMap);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        intake.intakePos();
        while (opModeIsActive()){
            intake.setIntakePower(intakePower);
            telemetry.addData("intake current", intake.intakeMotor.getCurrent());
            telemetry.addData("intake jammed", intake.isJammed());

            telemetry.update();
        }
    }
}
