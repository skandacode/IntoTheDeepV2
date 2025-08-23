package subsystems.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class OuttakeTesting extends LinearOpMode {
    public static double wristpos = 0.5;

    public static double flippos = 0.5;
    public static double clawpos = 0.8;
    public static double outtakeMotor1Power = 0;
    public static double outtakeMotor2Power = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo flip1 = hardwareMap.servo.get("flip1");
        Servo flip2 = hardwareMap.servo.get("flip2");
        DcMotor outtakeMotor1 = hardwareMap.dcMotor.get("outtakeMotor1");
        DcMotor outtakeMotor2 = hardwareMap.dcMotor.get("outtakeMotor2");

        waitForStart();

        while (opModeIsActive()){
            outtakeMotor1.setPower(outtakeMotor1Power);
            outtakeMotor2.setPower(outtakeMotor2Power);
            claw.setPosition(clawpos);
            wrist.setPosition(wristpos);
            flip1.setPosition(flippos);
            flip2.setPosition(1-flippos);
        }
    }
}
