package subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import subsystems.pathing.WayPoint;

public class Drivetrain {
    CachedMotorEx leftFront;
    CachedMotorEx leftBack;
    CachedMotorEx rightFront;
    CachedMotorEx rightBack;

    SimpleMotorFeedforward forwardFeedforward=new SimpleMotorFeedforward(0.08, 0.9);
    SimpleMotorFeedforward strafeFeedforward=new SimpleMotorFeedforward(0.2, 1);
    SimpleMotorFeedforward headingFeedforward=new SimpleMotorFeedforward(0.11, 1);


    Telemetry telemetry;
    FtcDashboard dashboard;


    public Pose2D position;

    private boolean ptoEngaged=false;

    CachedServo leftpto, rightpto;

    public Drivetrain(HardwareMap hwMap, Telemetry telemetry, FtcDashboard dashboard) {
        leftFront = new CachedMotorEx(hwMap, "frontleft");
        leftBack = new CachedMotorEx(hwMap, "backleft");
        rightFront = new CachedMotorEx(hwMap, "frontright");
        rightBack = new CachedMotorEx(hwMap, "backright");

        leftpto = new CachedServo(hwMap.servo.get("left_pto"));
        rightpto = new CachedServo(hwMap.servo.get("right_pto"));


        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        this.dashboard=dashboard;
        this.telemetry=telemetry;
    }
    public void setRawPowers(double frontleft, double frontright, double backleft, double backright){
        double maximum=Math.max(frontleft, frontright);
        maximum=Math.max(maximum, backleft);
        maximum=Math.max(maximum, backright);
        if (maximum>1){
            frontleft=frontleft/maximum;
            frontright=frontright/maximum;
            backleft=backleft/maximum;
            backright=backright/maximum;

        }
        leftFront.set(-frontleft);
        leftBack.set(-backleft);
        rightFront.set(frontright);
        rightBack.set(backright);
    }
    public void setWeightedPowers(double front, double strafe, double heading, double ptoPower){
        if (isPtoEngaged()){
            setRawPowers(ptoPower, ptoPower, ptoPower, ptoPower);
            return;
        }
        double weightedFront=forwardFeedforward.calculate(front);
        double weightedStrafe=strafeFeedforward.calculate(strafe);
        double weightedHeading=headingFeedforward.calculate(heading);

        setRawPowers(
                (weightedFront - weightedStrafe - weightedHeading),
                (weightedFront + weightedStrafe + weightedHeading),
                (weightedFront + weightedStrafe - weightedHeading),
                (weightedFront - weightedStrafe + weightedHeading)
        );
    }
    public void setWeightedPowers(double front, double strafe, double heading){
        setWeightedPowers(front, strafe, heading, 0);
    }
    public void setPtoEngaged(boolean engaged){
        ptoEngaged=engaged;
        if (ptoEngaged){
            leftpto.setPosition(0.4);
            rightpto.setPosition(0.5);
        }else{
            leftpto.setPosition(0.7);
            rightpto.setPosition(0.1);
        }
    }
    public boolean isPtoEngaged(){
        return ptoEngaged;
    }
    public void driveFieldCentric(double XPower, double YPower, double turnPower, double currHeading){
        double x = XPower * Math.cos(currHeading) + YPower * Math.sin(currHeading);
        double y = YPower * Math.cos(currHeading) - XPower * Math.sin(currHeading);
        setWeightedPowers(x, y, turnPower);
    }
    public void setTarget(WayPoint target){

    }
    public void update() {
        SparkFunOTOS.Pose2D rawposition= new SparkFunOTOS.Pose2D();
        position=new Pose2D(DistanceUnit.INCH, rawposition.x, rawposition.y, AngleUnit.DEGREES, rawposition.h);

        telemetry.addData("position", position.getX(DistanceUnit.INCH)+" "+position.getY(DistanceUnit.INCH)+" "+position.getHeading(AngleUnit.DEGREES));
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setFill("blue")
                .strokeCircle(position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH), 5)
                .strokeLine(position.getX(DistanceUnit.INCH), position.getY(DistanceUnit.INCH),
                        (Math.cos(position.getHeading(AngleUnit.RADIANS))*5)+ position.getX(DistanceUnit.INCH),
                        (Math.sin(position.getHeading(AngleUnit.RADIANS))*5)+ position.getY(DistanceUnit.INCH));

        dashboard.sendTelemetryPacket(packet);
    }
    public SparkFunOTOS.Pose2D getVelocity(){
        return new SparkFunOTOS.Pose2D();
    }
    public void updatePIDS(){

    }
    public boolean atTarget(){
        return true;
    }
    public void setPosition(Pose2D targetPosition){

    }
    public void calibrateIMU(){

    }
}
