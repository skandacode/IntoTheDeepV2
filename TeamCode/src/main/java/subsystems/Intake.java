package subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class Intake {
    private CachedMotorEx extendoMotor;
    private CachedMotorEx intakeMotor;
    private CachedServo intakeFlip1;
    private CachedServo intakeFlip2;
    private CachedServo cover;
    private TouchSensor limitSwitch;
    private CachedMotorEx backrightdt;
    private RevColorSensorV3 intakecolor;

    private PIDFController controller = new PIDFController(0.01, 0, 0, 0);

    private int currExtendoPos;
    private boolean retracted = true;

    public enum SampleColor {RED, BLUE, YELLOW, NONE}
    public static double distThreshold = 3;

    public Intake(HardwareMap hwMap){
        extendoMotor = new CachedMotorEx(hwMap, "extendo");
        intakeMotor = new CachedMotorEx(hwMap, "intake");
        backrightdt = new CachedMotorEx(hwMap, "backright");
        intakeFlip1 = new CachedServo(hwMap.servo.get("intakeFlip1"));
        intakeFlip2 = new CachedServo(hwMap.servo.get("intakeFlip2"));
        cover = new CachedServo(hwMap.servo.get("cover"));
        limitSwitch = hwMap.touchSensor.get("intakeEnd");
        intakecolor = hwMap.get(RevColorSensorV3.class, "color");
    }
    public void setExtendoPower(double power){
        extendoMotor.set(-power);
    }
    public void setIntakePower(double power){
        intakeMotor.set(-power);
    }
    public void setIntakeFlip(double pos){
        intakeFlip1.setPosition(pos);
        intakeFlip2.setPosition(1-pos);
    }
    public void setCover(boolean closed){
        if (closed){
            cover.setPosition(0.95);
        }else{
            cover.setPosition(0.05);
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
        else if (Math.abs(currExtendoPos-controller.getSetPoint())<10){
            power=0;
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
        setIntakeFlip(0.47);
        setCover(false);
        setIntakePower(0.3);
    }
    public void intakePos(int target){
        controller.setSetPoint(target);
        intakePos();
    }
    public void intakePos(){
        setIntakeFlip(0.65);
        setCover(true);
        setIntakePower(1);
    }
    public void eject(){
        setIntakeFlip(0.35);
    }
    public boolean isRetracted(){
        return retracted;
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
            int[] tweakedValues = new int[] {rgbValues[0]-20, rgbValues[1]-25, rgbValues[2]-30};
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
}