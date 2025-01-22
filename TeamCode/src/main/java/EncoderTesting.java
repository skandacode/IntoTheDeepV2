import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class EncoderTesting extends LinearOpMode {
    //code that logs the position of the motor to the telemetry
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotor motor = hardwareMap.dcMotor.get("backleft");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Encoder Pos", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
