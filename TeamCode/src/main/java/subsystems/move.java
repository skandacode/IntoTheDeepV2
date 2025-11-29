package subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@TeleOp
public class move extends LinearOpMode{
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    double backLeftPower;
    double backRightPower;


    public void moveBot(double forwardPower, double turnPower, double strafePower){
        frontLeft.setPower(forwardPower+turnPower+strafePower);
        frontRight.setPower(forwardPower-turnPower-strafePower);
        backLeft.setPower(forwardPower+turnPower-strafePower);
        backRight.setPower(forwardPower-turnPower+strafePower);
    }
    @Override
    public void runOpMode(){
        frontLeft = hardwareMap.get(DcMotor.class,("frontleft"));
        frontRight = hardwareMap.get(DcMotor.class,("frontright"));
        backLeft = hardwareMap.get(DcMotor.class,("backleft"));
        backRight = hardwareMap.get(DcMotor.class,("backright"));
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            double forwardPower = 0.3*-gamepad1.left_stick_y;
            double turnPower = 0.3*gamepad1.right_stick_x;
            double strafePower = 0.3*gamepad1.left_stick_x;

            moveBot(forwardPower,turnPower,strafePower);

        }
    }
}