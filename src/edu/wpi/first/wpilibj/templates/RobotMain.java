package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.io.BufferedReader;
import com.sun.squawk.microedition.io.FileConnection;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.io.Reader;
import javax.microedition.io.Connector;

public class RobotMain extends SimpleRobot {

    Compressor compressor = new Compressor(1, 1);
    RobotDrive chassis;
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    AxisCamera camera;
    Servo servoTest;
    DriverStation driverStation;

    final int frontLeft = 0;
    final int backLeft = 1;
    final int backRight = 2;
    final int frontRight = 3;

    double[] motorValues = new double[4];
    double[] offsetTrim = new double[4];
    double[] scaleTrim = new double[4];

    Talon fl;
    Talon bl;
    Talon fr;
    Talon br;

    private VisionProcessing visionProcessing;

    public RobotMain() {

    }

    public void robotInit() {
        camera = AxisCamera.getInstance("10.14.92.11");

        visionProcessing = new VisionProcessing();
        visionProcessing.init(camera);

        driverStation = DriverStation.getInstance();

        servoTest = new Servo(5);

        fl = new Talon(1);  //create the talons for each of the four wheels
        bl = new Talon(2);
        br = new Talon(3);
        fr = new Talon(4);

        compressor.start();
        chassis = new RobotDrive(fl, bl, fr, br);   //create the chassis with the motor drivers
        chassis.setExpiration(2);   // since this is a double it's probrably seconds not miliseconds
        chassis.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);   //invert the two motors so the robot drives correctly
        chassis.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
    }

    public void autonomous() {  //this method is called once when the robot is autonomous mode
        visionProcessing.autonomousInit();
        BinaryImage filteredImage;

        while (this.isAutonomous() && this.isEnabled()) {
            driveNowhere();

            try {
                filteredImage = visionProcessing.filterImage(camera.getImage());    //get an image and filter for vision targets
                visionProcessing.autonomousUpdate(filteredImage);   //process the filtered image and look for any potential targets
            } catch (Exception e) {
            }

            SmartDashboard.putBoolean("Target Hot", visionProcessing.target.Hot);
            Timer.delay(0.01);
        }
    }

    public void operatorControl() { //this method is called once when the robot is teleoperated mode
        chassis.setSafetyEnabled(true);
        SmartDashboard.putString("Alliance", driverStation.getAlliance().name);
        while (this.isOperatorControl() && this.isEnabled()) {
            SmartDashboard.putNumber("Mecanum X", getMecX());   //put the different motor and joystick values on the dashboard for debugging
            SmartDashboard.putNumber("Mecanum Y", getMecY());
            SmartDashboard.putNumber("Mecanum Rotation", getMecRot());
            SmartDashboard.putNumber("Front Left", fl.getSpeed());
            SmartDashboard.putNumber("Front Right", fr.getSpeed());
            SmartDashboard.putNumber("Back Left", bl.getSpeed());
            SmartDashboard.putNumber("Back Right", br.getSpeed());
            motorValues = mecanumDrive(getMecX(), getMecY(), getMecRot());  //
            //chassis.mecanumDrive_Cartesian(getMecX(), getMecY(), getMecRot(), 0);

            offsetTrim[frontLeft] = SmartDashboard.getNumber("Front Left Offset");  //get the offsets for the motors
            offsetTrim[frontRight] = SmartDashboard.getNumber("Front Right Offset");
            offsetTrim[backLeft] = SmartDashboard.getNumber("Back Left Offset");
            offsetTrim[backRight] = SmartDashboard.getNumber("Back Right Offset");

            scaleTrim[frontLeft] = SmartDashboard.getNumber("Front Left Scale");    //get the scaling for the motors
            scaleTrim[frontRight] = SmartDashboard.getNumber("Front Right Scale");
            scaleTrim[backLeft] = SmartDashboard.getNumber("Back Left Scale");
            scaleTrim[backRight] = SmartDashboard.getNumber("Back Right Scale");

            motorValues = trimValues(motorValues, scaleTrim, offsetTrim);   //scale and offset the motors
            driveMotors(motorValues);   //set the motors to the scaled and offset values
            
            if(SmartDashboard.getBoolean("Test file output", false)) {  //test writing to a file
                DataOutputStream file;
                FileConnection fc;
                try {
                    fc = (FileConnection)Connector.open("file:///test.txt", Connector.WRITE);
                    fc.create();
                    file = fc.openDataOutputStream();
                    file.writeUTF(SmartDashboard.getString("Data to write to file"));
                    file.flush();
                    file.close();
                    fc.close();
                } catch (IOException ex) {
                }
            }
            
            if(SmartDashboard.getBoolean("Test file input", false)) {   //test reaing from a file
                FileConnection fc;
                BufferedReader reader;
                try {
                    fc = (FileConnection)Connector.open("file:///test.txt", Connector.READ);
                    fc.create();
                    reader = new BufferedReader((Reader) fc);
                    String firstLine = reader.readLine();
                    SmartDashboard.putString("First line of the file", firstLine);  //put the first line of the file on the SmartDashboard
                    fc.close();
                } catch (IOException ex) {
                }
            }

            if (rightStick.getRawButton(3)) { //up
                servoTest.setAngle(-360);
            } else if (rightStick.getRawButton(2)) { //down
                servoTest.setAngle(360);
            }
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
        } catch (AxisCameraException ex) {
        } catch (NIVisionException ex) {
        }
        SmartDashboard.putBoolean("Target Hot", visionProcessing.target.Hot);

        while (this.isTest() && this.isEnabled()) { //keep the robot from returning errors
            driveNowhere();
            Timer.delay(0.1);
        }

    }

    private double getMecX() {  //get joystick values
        return deadZone(rightStick.getAxis(Joystick.AxisType.kX));
    }

    private double getMecY() {  //get joystick values
        return deadZone(rightStick.getAxis(Joystick.AxisType.kY));
    }

    private double getMecRot() {    //get joystick values
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

        double frn = 0;
        double fln = 0;
        double brn = 0;
        double bln = 0;

        fln = y + x + r;
        frn = y - x - r;
        bln = y - x + r;
        brn = y + x - r;

        double[] values = new double[4];

        values[frontRight] = frn;
        values[frontLeft] = fln;
        values[backRight] = brn;
        values[backLeft] = bln;
        return values;
    }

    private double[] trimValues(double[] orignalValues, double[] scale, double[] offset) {  //apply offset and scaling to the values
        double[] trimedValues = new double[4];

        for (int i = 0; i < 4; i++) {
            trimedValues[i] = orignalValues[i] * scale[i];
        }

        for (int i = 0; i < 4; i++) {
            trimedValues[i] = orignalValues[i] + offset[i];
        }

        return trimedValues;
    }

    private void driveMotors(double[] motorValues) {    //assign the values to the motors
        fl.set(maxAt1(motorValues[frontLeft]));
        bl.set(maxAt1(motorValues[backLeft]));
        br.set(maxAt1(motorValues[backRight]));
        fr.set(maxAt1(motorValues[frontRight]));
    }

    private double maxAt1(double n) {   //make the input value between 1 and -1
        return n < -1 ? -1 : (n > 1 ? 1 : n);
    }

    private void driveNowhere() {
        chassis.tankDrive(0, 0);
    }

}
