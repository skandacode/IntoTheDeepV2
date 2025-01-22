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

    PIDFController translationalControllerY=new PIDFController(0.07, 0, 0.007, 0);
    PIDFController translationalControllerX=new PIDFController(
            translationalControllerY.getP(),
            translationalControllerY.getI(),
            translationalControllerY.getD(),
            translationalControllerY.getF());
    PIDFController headingController=new PIDFController(0.8, 0, 0.03, 0);

    Telemetry telemetry;
    FtcDashboard dashboard;

    public SparkFunOTOS odometry;

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

        odometry = hwMap.get(SparkFunOTOS.class, "otos");
        odometry.calibrateImu();
        odometry.setLinearScalar(1.008);
        odometry.setAngularScalar(0.9896091044037605);
        odometry.setLinearUnit(DistanceUnit.INCH);
        odometry.setAngularUnit(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-7, -0.5, 90);
        odometry.setOffset(offset);
        odometry.resetTracking();

        odometry.setPosition(new SparkFunOTOS.Pose2D());

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
        translationalControllerX.setSetPoint(target.getPosition().getX(DistanceUnit.INCH));
        translationalControllerY.setSetPoint(target.getPosition().getY(DistanceUnit.INCH));
        headingController.setSetPoint(target.getPosition().getHeading(AngleUnit.RADIANS));

        translationalControllerX.setTolerance(target.getTolerance().getX(DistanceUnit.INCH));
        translationalControllerY.setTolerance(target.getTolerance().getY(DistanceUnit.INCH));
        headingController.setTolerance(target.getTolerance().getHeading(AngleUnit.RADIANS));
    }
    public void update() {
        SparkFunOTOS.Pose2D rawposition= odometry.getPosition();
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
        return odometry.getVelocity();
    }
    public void updatePIDS(){
        double heading=position.getHeading(AngleUnit.RADIANS);
        while (Math.abs(heading-headingController.getSetPoint())>Math.PI){
            if (heading<headingController.getSetPoint()){
                heading=heading+2*Math.PI;
            }else{
                heading=heading-2*Math.PI;
            }
        }
        double x_velo=translationalControllerX.calculate(position.getX(DistanceUnit.INCH));
        double y_velo=translationalControllerY.calculate(position.getY(DistanceUnit.INCH));
        double heading_velo=headingController.calculate(heading);
        telemetry.addData("velocity x", x_velo);
        telemetry.addData("velocity y", y_velo);
        telemetry.addData("velocity heading", heading_velo);

        if (Math.abs(position.getY(DistanceUnit.INCH)-translationalControllerY.getSetPoint())<0.5){
            y_velo=0;
        }
        if (Math.abs(position.getX(DistanceUnit.INCH)-translationalControllerX.getSetPoint())<0.5){
            x_velo=0;
        }
        if (Math.abs(position.getHeading(AngleUnit.DEGREES)-headingController.getSetPoint())<0.5){
            heading_velo=0;
        }
        driveFieldCentric(x_velo, y_velo,heading_velo, heading);
    }
    public boolean atTarget(){
        return translationalControllerX.atSetPoint() && translationalControllerY.atSetPoint() && headingController.atSetPoint();
    }
    public void setPosition(Pose2D targetPosition){

        odometry.setPosition(new SparkFunOTOS.Pose2D(targetPosition.getX(DistanceUnit.INCH),
                targetPosition.getY(DistanceUnit.INCH), targetPosition.getHeading(AngleUnit.DEGREES)));
    }
    public void calibrateIMU(){
        odometry.calibrateImu();
    }
}
