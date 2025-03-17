package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
@Config
public class Intake {
    CachedMotorEx extendoMotor;
    CachedMotorEx intakeMotor;
    private CachedServo intakeFlip1;
    private CachedServo intakeFlip2;
    private CachedServo cover;
    public TouchSensor limitSwitch;
    private CachedMotorEx backrightdt;
    private RevColorSensorV3 intakecolor;

    private AnalogInput flipAnalog;

    public static double kP = 0.01;
    public static double kD = 0.0007;
    public static double kF=0.09;

    private PIDFController controller = new PIDFController(kP, 0, kD, 0);

    private int currExtendoPos;
    private boolean retracted = true;

    public enum SampleColor {RED, BLUE, YELLOW, NONE}
    public static double distThreshold = 4.9;

    public double intakePower=0;

    public static double k=0.05;
    private double filtered=50;

    public Intake(HardwareMap hwMap){
        extendoMotor = new CachedMotorEx(hwMap, "extendo");
        intakeMotor = new CachedMotorEx(hwMap, "intake");
        backrightdt = new CachedMotorEx(hwMap, "backright");
        intakeFlip1 = new CachedServo(hwMap.servo.get("intakeFlip1"));
        intakeFlip2 = new CachedServo(hwMap.servo.get("intakeFlip2"));
        cover = new CachedServo(hwMap.servo.get("cover"));
        limitSwitch = hwMap.touchSensor.get("intakeEnd");
        intakecolor = hwMap.get(RevColorSensorV3.class, "color");

        flipAnalog = hwMap.analogInput.get("intake_flip_analog");

        intakeMotor.setCurrentLimit(8.5);
    }
    public void setExtendoPower(double power){
        extendoMotor.set(-power);
    }
    public void setIntakePower(double power){
        intakeMotor.set(-power);
        intakePower=power;
    }
    public void setIntakeFlip(double pos){
        intakeFlip1.setPosition(pos);
        intakeFlip2.setPosition(1-pos);
    }
    public void setCover(boolean closed){
        if (closed){
            cover.setPosition(0.93);
        }else{
            cover.setPosition(0.1);
        }
    }
    public int getExtendoMotorPosition(){
        return -backrightdt.getCurrentPosition();
    }
    public int getCachedExtendoPos(){
        return currExtendoPos;
    }
    public void update(){
        currExtendoPos = getExtendoMotorPosition();
        double power = controller.calculate(currExtendoPos);
        retracted = limitSwitch.isPressed();
        if (controller.getSetPoint() == 0){
            if (!retracted){
                power=-1;
                //System.out.println("Retracting and not pressed");

            }else{
                power=-0.4;
                if (currExtendoPos!=0) {
                    backrightdt.resetEncoder();
                    //System.out.println("Resetting encoder and setting power to -0.4");
                }else{
                    //System.out.println("setting power to -0.4 without resetting encoder");
                }
            }
        }
        else if (Math.abs(currExtendoPos-controller.getSetPoint())<5){
            power=0;
        }else{
            if (currExtendoPos<controller.getSetPoint()){
                power+=kF;
            }else{
                power-=kF;
            }
        }
        setExtendoPower(power);
    }
    public void setTargetPos(int pos){
        if (pos != controller.getSetPoint()) {
            controller.setSetPoint(pos);
            controller.reset();
        }
    }
    public void transferPos(){
        controller.setSetPoint(0);
        setIntakeFlip(0.43);
        setCover(false);
        //setIntakePower(0.3);
    }
    public void intakePos(int target){
        controller.setSetPoint(target);
        intakePos();
    }
    public void intakePos(){
        setIntakeFlip(0.63);
        setCover(true);
        setIntakePower(1);
    }
    public void intakeEject(){
        setIntakeFlip(0.63);
    }
    public void eject(){

        setIntakeFlip(0.3);
        setIntakePower(0.5);
    }
    public boolean isRetracted(){
        return limitSwitch.isPressed();
    }
    public int[] getRawSensorValues() {
        return new int[]{intakecolor.red(), intakecolor.green(), intakecolor.blue()}; // Return RGB as an array
    }
    public double getDistance(){
        return intakecolor.getDistance(DistanceUnit.CM);
    }
    public SampleColor getColor(){
        if (isSampleIntaked()){
            int[] rgbValues = getRawSensorValues();
            System.out.println(Arrays.toString(rgbValues));
            int[] tweakedValues = new int[] {rgbValues[0]-20, rgbValues[1]-35, rgbValues[2]-30};
            if (tweakedValues[0]>tweakedValues[1] && tweakedValues[0]>tweakedValues[2]){
                System.out.println(Arrays.toString(tweakedValues)+" Red");
                return SampleColor.RED;
            }
            if (tweakedValues[1]>tweakedValues[0] && tweakedValues[1]>tweakedValues[2]){
                System.out.println(Arrays.toString(tweakedValues)+" Yellow");
                return SampleColor.YELLOW;
            }
            if (tweakedValues[2]>tweakedValues[1] && tweakedValues[2]>tweakedValues[0]){
                System.out.println(Arrays.toString(tweakedValues)+" Blue");
                return SampleColor.BLUE;
            }
            System.out.println("Failed all checks but somethign is there");
        }else{
            return SampleColor.NONE;
        }
        System.out.println("Possible intake hang");
        return SampleColor.NONE;
    }
    public boolean isSampleIntaked(){
        return getDistance()<distThreshold;
    }
    public double getTarget(){
        return controller.getSetPoint();
    }
    public boolean isJammed(){
        return intakeMotor.isOverCurrent();
    }
    public double getFlipAnalog(){
        return flipAnalog.getVoltage();
    }
}