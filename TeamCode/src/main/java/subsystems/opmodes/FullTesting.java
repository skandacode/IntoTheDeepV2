package subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp
public class FullTesting extends LinearOpMode {
    public static double wristpos = 0.5;
    public static double flippos = 0.5;
    public static double railpos = 0.7;
    public static double clawpos = 0.8;
    public static double outtakeMotorPower = 0;
    public static double latchpos = 0.5;
    public static double intakeflippos = 0.5;
    public static double coverpos = 0.5;
    public static double motorpower = 0;
    public static double extendopower = 0;
    public static double intakepower = 0;

    public static double PTOLEFT = 0.7;
    public static double PTORIGHT = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Servo rail = hardwareMap.servo.get("rail");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");
        Servo flip1 = hardwareMap.servo.get("flip1");
        Servo flip2 = hardwareMap.servo.get("flip2");
        Servo intakeflip= hardwareMap.servo.get("intakeFlip");
        Servo cover = hardwareMap.servo.get("cover");
        Servo rightlatch = hardwareMap.servo.get("latch_right");
        Servo leftlatch = hardwareMap.servo.get("latch_left");
        Servo rightpto = hardwareMap.servo.get("right_pto");
        Servo leftpto = hardwareMap.servo.get("left_pto");

        TouchSensor intakeEnd = hardwareMap.touchSensor.get("intakeEnd");
        TouchSensor outtakeEnd = hardwareMap.touchSensor.get("outtakeEnd");


        DcMotorEx outtakeMotor1 = (DcMotorEx) hardwareMap.dcMotor.get("outtakeMotor1");
        DcMotorEx outtakeMotor2 = (DcMotorEx) hardwareMap.dcMotor.get("outtakeMotor2");
        DcMotor backleft = hardwareMap.dcMotor.get("backleft");
        DcMotor backright = hardwareMap.dcMotor.get("backright");
        DcMotor frontleft = hardwareMap.dcMotor.get("frontleft");
        DcMotor frontright = hardwareMap.dcMotor.get("frontright");
        DcMotorEx extendo = (DcMotorEx) hardwareMap.dcMotor.get("extendo");
        DcMotor intake = hardwareMap.dcMotor.get("intake");

        waitForStart();

        while (opModeIsActive()){
            outtakeMotor1.setPower(outtakeMotorPower);
            outtakeMotor2.setPower(-outtakeMotorPower);
            claw.setPosition(clawpos);
            wrist.setPosition(wristpos);
            flip1.setPosition(flippos);
            flip2.setPosition(1-flippos);
            rail.setPosition(railpos);
            backleft.setPower(motorpower);
            backright.setPower(-1*motorpower);
            frontleft.setPower(motorpower);
            frontright.setPower(-1*motorpower);
            intake.setPower(intakepower);
            extendo.setPower(extendopower);
            rightlatch.setPosition(latchpos);
            leftlatch.setPosition(1-latchpos);
            /*
            if (pto_engaged){
                leftpto.setPosition(0.43);
                rightpto.setPosition(0.5);
            }else{
                leftpto.setPosition(0.7);
                rightpto.setPosition(0.1);
            }
            */
            leftpto.setPosition(PTOLEFT);
            rightpto.setPosition(PTORIGHT);

            cover.setPosition(coverpos);
            intakeflip.setPosition(intakeflippos);
            telemetry.addData("outtakeMotor1 pos", outtakeMotor1.getCurrentPosition());
            telemetry.addData("extendo pos", extendo.getCurrentPosition());

            telemetry.addData("frontleft pos", frontleft.getCurrentPosition());
            telemetry.addData("backleft pos", backleft.getCurrentPosition());
            telemetry.addData("frontright pos", frontright.getCurrentPosition());
            telemetry.addData("backright pos", backright.getCurrentPosition());



            telemetry.addData("outtakeMotor1 current", outtakeMotor1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("outtakeMotor2 current", outtakeMotor2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("extendo current", extendo.getCurrent(CurrentUnit.AMPS));

            telemetry.addData("intake end", intakeEnd.isPressed());
            telemetry.addData("outtake end", outtakeEnd.isPressed());

            telemetry.update();
        }
    }
}
