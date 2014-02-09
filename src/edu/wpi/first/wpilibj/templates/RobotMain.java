package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class RobotMain extends SimpleRobot {
    
    
    public static int ROTARY_ERROR = 0;
    public static int ROTARY_LOAD = 1;
    public static int ROTARY_LOW_GOAL = 2;
    public static int ROTARY_TRUSS = 3;
    public static int ROTARY_HIGH_GOAL = 4;

    Compressor compressor = new Compressor(1, 1);
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    Joystick manipulatorStick = new Joystick(3);
    AxisCamera camera;
    DriverStation driverStation;
    DriverStationEnhancedIO driverStationIO;
    
    Talon fl;   //the motor drivers
    Talon bl;
    Talon fr;
    Talon br;
    
    Talon pickupRoller;
    
    Solenoid trigger;
    DoubleSolenoid pickupFrame;
    DoubleSolenoid catapult;

    private VisionProcessing visionProcessing;
    
    private boolean triggerButtonDown = false;
    private int pos = ROTARY_ERROR;
    private int lastPos = ROTARY_ERROR;

    public RobotMain() {

    }

    public void robotInit() {
        camera = AxisCamera.getInstance("10.14.92.11");

        visionProcessing = new VisionProcessing(camera);

        driverStation = DriverStation.getInstance();
        
        driverStationIO = driverStation.getEnhancedIO();

        fl = new Talon(1);  //create the talons for each of the four wheels
        bl = new Talon(2);
        br = new Talon(3);
        fr = new Talon(4);
        
        pickupRoller = new Talon(5);
        
        trigger = new Solenoid(0);
        pickupFrame = new DoubleSolenoid(1, 2);
        catapult = new DoubleSolenoid(1, 2);

        compressor.start();
    }

    public void autonomous() {  //this method is called once when the robot is autonomous mode

    }

    public void operatorControl() { //this method is called once when the robot is teleoperated mode
        SmartDashboard.putString("Alliance", driverStation.getAlliance().name);
        while (this.isOperatorControl() && this.isEnabled()) {
            mecanumDrive(getMecX(), getMecY(), getMecRot());
            double v = deadZone(manipulatorStick.getAxis(Joystick.AxisType.kY));
            pickupRoller.set(v>0?1:v<0?-1:0);

            pos = ROTARY_ERROR;
            
            try {
                pos = driverStationIO.getDigital(6)?ROTARY_LOAD:
                        driverStationIO.getDigital(8)?ROTARY_LOW_GOAL:
                        driverStationIO.getDigital(10)?ROTARY_TRUSS:
                        driverStationIO.getDigital(12)?ROTARY_HIGH_GOAL:
                        ROTARY_ERROR;
            } catch (EnhancedIOException ex) {
            }
            if(lastPos!=pos){
                if(pos == ROTARY_LOW_GOAL){
                    catapult.set(Value.kReverse);
                }else if(pos == ROTARY_TRUSS){
                    catapult.set(Value.kReverse);
                }else if(pos == ROTARY_HIGH_GOAL){
                    catapult.set(Value.kForward);
                }else{
                    catapult.set(Value.kReverse);
                    trigger.set(false);
                }
            }
            lastPos = pos;
            SmartDashboard.putNumber("Position", pos);
            
            boolean down = manipulatorStick.getRawButton(1);
            boolean pressed = !triggerButtonDown && down;
            boolean released = triggerButtonDown && !down;
            
            
            if(pressed){
                if(pos != ROTARY_LOAD){
                    trigger.set(true);
                }
                new CatapultThread(pos, this).start();
                
            }
            if(released){
                if(pos != ROTARY_LOAD){
                    trigger.set(false);
                    catapult.set(Value.kReverse);
                }
            }
            
            triggerButtonDown = down;

            Timer.delay(0.01);  //do not run the loop to fast
        }
    }

    public void disabled() {

    }

    public void test() {    //this method is called once when the robot is test mode
        visionProcessing.autonomousInit();
        BinaryImage filteredImage;

        try {
            filteredImage = visionProcessing.filterImageTest(camera.getImage());
            visionProcessing.autonomousUpdate(filteredImage);
        } catch (Exception ex) {
        }

        while (this.isTest() && this.isEnabled()) { //keep the robot from returning errors
            driveNowhere();
            Timer.delay(0.1);
        }
    }

    public double getMecX() {  //get joystick values
        return deadZone(rightStick.getAxis(Joystick.AxisType.kX));
    }

    public double getMecY() {  //get joystick values
        return deadZone(rightStick.getAxis(Joystick.AxisType.kY));
    }

    public double getMecRot() {    //get joystick values
        return deadZone(leftStick.getAxis(Joystick.AxisType.kX));
    }

    private double deadZone(double value) { //apply a dead zone to a value
        return (abs(value) < .1) ? 0 : value;
    }

    private double abs(double value) {  //absoulte value
        return value < 0 ? -value : value;
    }

    private double[] mecanumDrive(double x, double y, double r) {   // find the values for the motors based on x, y and rotation values
        y = -y;

        double frn = y + x + r;
        double fln = y - x - r;
        double brn = y - x + r;
        double bln = y + x - r;

        double[] values = new double[4];

        fr.set(maxAt1(frn));
        fl.set(maxAt1(fln));
        br.set(maxAt1(brn));
        bl.set(maxAt1(bln));
        return values;
    }

    private double maxAt1(double n) {   //make the input value between 1 and -1
        return n < -1 ? -1 : (n > 1 ? 1 : n);
    }

    private void driveNowhere() {
        fl.set(0);
        bl.set(0);
        br.set(0);
        fr.set(0);
    }
}
