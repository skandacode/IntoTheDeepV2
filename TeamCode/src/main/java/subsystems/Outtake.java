package subsystems;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Outtake {
    private CachedMotorEx outtakeMotor1;
    private CachedMotorEx outtakeMotor2;
    private CachedServo rail;
    private CachedServo claw;
    private CachedServo wrist;
    private CachedServo flip1;
    private CachedServo flip2;

    private TouchSensor limitSwitch;

    private AnalogInput flipAnalog;

    private PIDFController controller = new PIDFController(0.01, 0, 0, 0);

    private int currMotorPos;
    private boolean retracted = true;
    public double currPower=0;

    public Outtake(HardwareMap hwMap){
        outtakeMotor1 = new CachedMotorEx(hwMap, "outtakeMotor1");
        outtakeMotor2 = new CachedMotorEx(hwMap, "outtakeMotor2");
        outtakeMotor2.setRunMode(Motor.RunMode.RawPower);
        outtakeMotor1.setRunMode(Motor.RunMode.RawPower);

        rail = new CachedServo(hwMap.servo.get("rail"));
        claw = new CachedServo(hwMap.servo.get("claw"));
        wrist = new CachedServo(hwMap.servo.get("wrist"));
        flip1 = new CachedServo(hwMap.servo.get("flip1"));
        flip2 = new CachedServo(hwMap.servo.get("flip2"));

        limitSwitch = hwMap.touchSensor.get("outtakeEnd");

        flipAnalog = hwMap.analogInput.get("flip_analog");

        currMotorPos = 0;

        controller.setTolerance(10);
        controller.setSetPoint(0);
        outtakeMotor1.setCurrentLimit(6);

    }
    public void update(){
        currMotorPos = getOuttakePosition();
        double power = controller.calculate(currMotorPos);
        retracted = limitSwitch.isPressed();
        if (controller.getSetPoint() == 0){
            if (!retracted){
                power=-1;
            }else{
                power=-0.1;
                outtakeMotor1.resetEncoder();
            }
        }
        else if (Math.abs(currMotorPos-controller.getSetPoint())<10){
            power=0.25;
        }
        else{
            if (currMotorPos<controller.getSetPoint()){
                power+=0.4;
            }else{
                power+=0.12;
            }
        }
        setMotors(power);
    }
    public void setMotors(double power){
        currPower=power;
        outtakeMotor1.set(-power);
        outtakeMotor2.set(power);
    }
    //functions that set servos to positions
    public void setRail(double pos){
        rail.setPosition(pos+0.01);
    }
    public void setClaw(double pos){
        claw.setPosition(pos);
    }
    public void setWrist(double pos){
        wrist.setPosition(pos);
    }
    public void setFlip(double pos){
        flip1.setPosition(pos);
        flip2.setPosition(1-pos);
    }
    public int getOuttakePosition(){
        return -outtakeMotor1.getCurrentPosition();
    }
    public void setTargetPos(int pos){
        if (pos != controller.getSetPoint()){
            controller.setSetPoint(pos);
            controller.reset();
        }
    }
    public void transferPos(){
        closeClaw();
        setRail(0.445);
        setFlip(0.7);
        setWrist(0.1);
        setTargetPos(0);
    }
    public void waitPos(){

    }
    public void partialSampleFlip(){
        closeClaw();
        setRail(0.35);
        setFlip(0.4);
        setWrist(0.55);
        setTargetPos(1000);
    }
    public void sampleScore(){
        closeClaw();
        setRail(0.23);
        setFlip(0.03);
        setWrist(0.5);
        setTargetPos(1250);
    }
    public void specHold(){
        closeClaw();
        setRail(0.52);
        setFlip(0.71);
        setWrist(0.55);
        setTargetPos(600);

    }
    public void specScore(){
        closeClaw();
        setRail(0.56);
        setFlip(0.6);
        setWrist(0.6);
        setTargetPos(700);

    }
    public int getCachedPos(){
        //return currMotorPos;
        return getOuttakePosition();
    }
    public void openClaw(){
        claw.setPosition(0.72);
        //System.out.println("Opened claw");
    }
    public void openClawWide(){
        claw.setPosition(0.52);
        //System.out.println("Opened wide claw");
    }
    public void closeClaw(){
        claw.setPosition(0.88);
        //System.out.println("Closed claw");
    }
    public void closeClawOverfill(){
        claw.setPosition(0.84);
        //System.out.println("Closed claw");
    }
    public double getSetPoint() {
        return controller.getSetPoint();
    }
    public boolean isRetracted() {
        return limitSwitch.isPressed();
    }
    public boolean atTarget(){
        return controller.atSetPoint();
    }
    public double getFlipAnalog(){
        return flipAnalog.getVoltage();
    }
}
