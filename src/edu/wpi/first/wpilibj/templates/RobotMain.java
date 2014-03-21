package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotMain extends SimpleRobot {

    public static int ROTARY_LOAD = 1;  //The values for the different modes
    public static int ROTARY_LOW_GOAL = 2;
    public static int ROTARY_TRUSS = 3;
    public static int ROTARY_HIGH_GOAL = 4;

    Compressor compressor = new Compressor(1, 1);   //Initalize the compressor
    Joystick leftStick = new Joystick(1);   //Joystick for rotation of the robot
    Joystick rightStick = new Joystick(2);  //Joystick for the xy position of the robot
    Joystick manipulatorStick = new Joystick(3);    //Joystick for the manipulator
    AxisCamera camera;  //Create the camera
    DriverStation driverStation;    //Create the driver station
    
    DigitalInput diA;
    DigitalInput diB;

    Talon fl;   //The motor drivers
    Talon bl;
    Talon fr;
    Talon br;

    Talon pickupRoller; //The roller on the frame to pickup the ball

    Relay trigger;  //The valve for the trigger release
    Relay pickupFrame;  //The valve to move the frame with the roller to pickup the ball up or down
    Relay catapult; //The valve to move the catapult up or down

    private VisionProcessing visionProcessing;  //The vision processing object

    private boolean triggerButtonDown = false;  //State of the trigger
    private int pos = ROTARY_LOAD;  //The state of the knob
    private int lastPos = ROTARY_LOAD;  //Used to check if the knob has changed

    public RobotMain() {

    }

    public void robotInit() {
        camera = AxisCamera.getInstance("10.14.92.11"); //Initalize the camera

        visionProcessing = new VisionProcessing();  //Initalize the vision processing

        driverStation = DriverStation.getInstance();    //Initalize the driver station
        
        diA = new DigitalInput(13);
        diB = new DigitalInput(14);

        fl = new Talon(2);  //Initalize the talons for each of the four wheels
        bl = new Talon(1);
        br = new Talon(3);
        fr = new Talon(4);

        pickupRoller = new Talon(5);    //Initalize the roller

        trigger = new Relay(4); //Initalize the trigger
        pickupFrame = new Relay(3); //Initalize the frame
        catapult = new Relay(2);    //Initalize the frame

        compressor.start(); //Start the compressor
    }

    public void autonomous() {  //This method is called once when the robot is autonomous mode
        int state = 1;      //1 is single shoot, 2 is double
        
        boolean a = diA.get();
        boolean b = diB.get();
        
        if(a){
            if(b){
                state = 1; // Shoot then move
            }else{
                state = 2; // Shoot, load, shoot, move
            }
        }else{
            if(b){
                state = 3; // Just move
            }else{
                state = 0; // Nothing
            }
        }

        if (state == 1 || state == 2) {
            catapult.set(Relay.Value.kForward); //Pre-charge the catapult
            pickupFrame.set(Value.kReverse);    //Make sure the frame with the rollers is out of the way
            Timer.delay(1);    //Wait
            trigger.set(Relay.Value.kForward);  //Engauge the trigger
            Timer.delay(1);    //Wait
            trigger.set(Relay.Value.kOff);  //Latch the trigger
            catapult.set(Relay.Value.kReverse); //Pull the catapult back down
        }
        if (state == 2) {
            Timer.delay(1);
            pickupFrame.set(Value.kForward);
            pickupRoller.set(-1);
            Timer.delay(2);    //Wait
            pickupRoller.set(0);
            pickupFrame.set(Value.kReverse);
            Timer.delay(1);

            catapult.set(Relay.Value.kForward); //Pre-charge the catapult
            pickupFrame.set(Value.kReverse);    //Make sure the frame with the rollers is out of the way
            Timer.delay(1);    //Wait
            trigger.set(Relay.Value.kForward);  //Engauge the trigger
            Timer.delay(1);    //Wait
            trigger.set(Relay.Value.kOff);  //Latch the trigger
            catapult.set(Relay.Value.kReverse); //Pull the catapult back down
        }
        if (state == 1 || state == 2 || state == 3) {
            moveAll(1); //Move forward for the mobility points
            Timer.delay(.5);    //Wait
            moveAll(0); //Stop so we do not crash into the wall
        }

    }

    public void operatorControl() { //This method is called once when the robot is teleoperated mode
        SmartDashboard.putString("Alliance", driverStation.getAlliance().name); //Put the data to smart dashboard only once
        SmartDashboard.putString("Trigger Relay: ", ValueToString(trigger.get()));
        SmartDashboard.putString("Pickup Frame Relay: ", ValueToString(pickupFrame.get()));
        SmartDashboard.putString("Catapult Relay: ", ValueToString(catapult.get()));
        SmartDashboard.putBoolean("Compressor Enabled", compressor.enabled());

        pickupFrame.set(Value.kForward);    //Pull the frame in

        while (this.isOperatorControl() && this.isEnabled()) {  //Everything in here happens forever
            mecanumDrive(getMecX(), getMecY(), getMecRot());    //Drive the robot
            double v = deadZone(manipulatorStick.getAxis(Joystick.AxisType.kY), .5);    //Should the pickup rollers do anything
            pickupRoller.set(v > 0.3 ? -1 : v < -0.3 ? 1 : 0);
            SmartDashboard.putNumber("Pickup roller motor", (v > 0.3 ? -1 : v < -0.3 ? 1 : 0));
            SmartDashboard.putNumber("Pickup roller joystick", v);

            if (manipulatorStick.getRawButton(3)) { //Move the frame in when the up button is pressed
                pickupFrame.set(Value.kForward);
            } else if (manipulatorStick.getRawButton(2)) {  //Move the frame out when the down button is pressed
                pickupFrame.set(Value.kReverse);
            }

            pos = ROTARY_LOAD;  //Default to the "off" position

            SmartDashboard.putNumber("Knob position", 0);

            SmartDashboard.putBoolean("is 1 (Off)", !driverStation.getDigitalIn(1));    //Display the status of the digital I/O
            SmartDashboard.putBoolean("is 2 (Low)", !driverStation.getDigitalIn(2));
            SmartDashboard.putBoolean("is 3 (High)", !driverStation.getDigitalIn(3));
            SmartDashboard.putBoolean("is 4 (Truss)", !driverStation.getDigitalIn(4));

            try {   //Determine the position of the knob
                if (!driverStation.getDigitalIn(1)) {
                    pos = ROTARY_LOAD;
                    SmartDashboard.putNumber("Knob position", 1);
                } else if (!driverStation.getDigitalIn(2)) {
                    pos = ROTARY_LOW_GOAL;
                    SmartDashboard.putNumber("Knob position", 2);
                } else if (!driverStation.getDigitalIn(3)) {
                    pos = ROTARY_HIGH_GOAL;
                    SmartDashboard.putNumber("Knob position", 3);
                } else if (!driverStation.getDigitalIn(4)) {
                    pos = ROTARY_TRUSS;
                    SmartDashboard.putNumber("Knob position", 4);
                }
            } catch (Exception e) {
                SmartDashboard.putString("Error: ", e.getMessage());
            }

            if (lastPos != pos) {   //When the position is changed...
                if (pos == ROTARY_LOW_GOAL) {
                    catapult.set(Relay.Value.kReverse); //Pull the catapult down
                } else if (pos == ROTARY_TRUSS) {
                    catapult.set(Relay.Value.kReverse); //Pull the catapult down
                } else if (pos == ROTARY_HIGH_GOAL) {
                    catapult.set(Relay.Value.kForward); //Pre-charge the catapult
                } else {    //If in the ROTARY_LOAD mode or somethign has gone horribly wrong
                    catapult.set(Relay.Value.kReverse); //Switch the catapult off
                    trigger.set(Relay.Value.kOff);
                }
            }

            lastPos = pos;  //Reset the previous position of the knob
            SmartDashboard.putNumber("Position", pos);  //Update the position of the knob

            boolean down = manipulatorStick.getRawButton(1);    //Is the trigger button pressed
            boolean pressed = !triggerButtonDown && down;   //Has the trigger button gone from up to down
            boolean released = triggerButtonDown && !down;  //has the trigger button gone from up to down

            if (pressed) {
                if (pos != ROTARY_LOAD) {   //If we should be able to shoot
                    pickupFrame.set(Value.kReverse);    //Do not shoot with the frame in the way
                    Timer.delay(.2);
                    trigger.set(Relay.Value.kForward);  //Engauge the trigger
                    new CatapultThread(pos, this).start();  //Do the necessary catapult stuff
                }
            }

            if (released) {
                if (pos != ROTARY_LOAD) {   //If we are done shooting
                    trigger.set(Relay.Value.kOff);  //Latch the trigger
                    catapult.set(Relay.Value.kReverse); //Pull the catapult down
                    Timer.delay(.2);
                    pickupFrame.set(Value.kForward);
                }
            }

            triggerButtonDown = down;   //Update the trigger button state

            Timer.delay(0.01);  //Do not run the loop to fast
        }
    }

    public void disabled() {

    }

    public void test() {    //This method is called once when the robot is test mode
        visionProcessing.autonomousInit();
        BinaryImage filteredImage;

        try {
            filteredImage = visionProcessing.filterImageTest(camera.getImage());
            visionProcessing.autonomousUpdate(filteredImage);
        } catch (Exception ex) {
            SmartDashboard.putString("Error: ", ex.getMessage());
        }

        while (this.isTest() && this.isEnabled()) { //Keep the robot from returning errors
            moveAll(0);
            Timer.delay(0.1);
        }
    }

    public double getMecX() {   //Get joystick values
        return deadZone(rightStick.getAxis(Joystick.AxisType.kX));
    }

    public double getMecY() {   //Get joystick values
        return deadZone(rightStick.getAxis(Joystick.AxisType.kY));
    }

    public double getMecRot() { //Get joystick values
        return deadZone(leftStick.getAxis(Joystick.AxisType.kX));
    }

    private double deadZone(double value, double amount) {  //Apply a dead zone to a value
        return (abs(value) < amount) ? 0 : value;
    }

    private double deadZone(double value) {
        return deadZone(value, .2);
    }

    private double abs(double value) {  //Absoulte value
        return value < 0 ? -value : value;
    }

    private double[] mecanumDrive(double x, double y, double r) {   //Find the values for the motors based on x, y and rotation values
        x = -x;
        y = -y;
        r = -r;

        double frn = y + x + r;
        double fln = y - x - r;
        double brn = y - x + r;
        double bln = y + x - r;

        double[] values = new double[4];

        move(fln, frn, bln, brn);
        return values;
    }

    private double maxAt1(double n) {   //Make the input value between 1 and -1
        return n < -1 ? -1 : (n > 1 ? 1 : n);
    }

    private String ValueToString(Value v) {
        switch (v.value) {
            case 2:
                return "Forward";
            case 3:
                return "Reverse";
            case 1:
                return "On";
            case 0:
                return "Off";
        }
        return "Error";
    }

    private void moveAll(double i) {
        move(i, i, i, i);
    }

    private void move(double flv, double frv, double blv, double brv) {
        fr.set(-maxAt1(frv));
        fl.set(maxAt1(flv));
        br.set(-maxAt1(brv));
        bl.set(maxAt1(blv));
    }
}
