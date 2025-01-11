package subsystems.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTesting extends LinearOpMode {
    public static double test1 = 0.5;
    public static double test2 = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo testServo1 = hardwareMap.servo.get("testServo1");
        Servo testServo2 = hardwareMap.servo.get("testServo2");

        waitForStart();

        while (opModeIsActive()){
            testServo2.setPosition(test2);
            testServo1.setPosition(test1);
        }
    }
}
