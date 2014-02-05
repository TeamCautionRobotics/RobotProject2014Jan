package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.io.BufferedReader;
import com.sun.squawk.microedition.io.FileConnection;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.smartdashboard.*;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.Reader;
import javax.microedition.io.Connector;

public class RobotMain extends SimpleRobot {

    Compressor compressor = new Compressor(1, 1);
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    AxisCamera camera;
    DriverStation driverStation;
    DriverStationEnhancedIO driverStationIO;

    final int frontLeft = 0;
    final int backLeft = 1;
    final int backRight = 2;
    final int frontRight = 3;

    double[] motorValues = new double[4];
    double[] offsetTrim = new double[]{0, 0, 0, 0};
    double[] scaleTrim = new double[]{1, 1, -1, -1};    //invert the two right motors so the robot drives correctly

    Talon fl;   //the motor drivers
    Talon bl;
    Talon fr;
    Talon br;

    private VisionProcessing visionProcessing;
    private DashboardCommunication dashboardCommunication;

    private boolean writeToFile = false;
    private boolean readFromFile = false;

    public RobotMain() {

    }

    public void robotInit() {
        camera = AxisCamera.getInstance("10.14.92.11");

        visionProcessing = new VisionProcessing(camera);
        dashboardCommunication = new DashboardCommunication(this);

        driverStation = DriverStation.getInstance();

        driverStationIO = driverStation.getEnhancedIO();

        fl = new Talon(1);  //create the talons for each of the four wheels
        bl = new Talon(2);
        br = new Talon(3);
        fr = new Talon(4);

        compressor.start();
    }

    public void autonomous() {  //this method is called once when the robot is autonomous mode

    }

    public void operatorControl() { //this method is called once when the robot is teleoperated mode
        SmartDashboard.putString("Alliance", driverStation.getAlliance().name);
        while (this.isOperatorControl() && this.isEnabled()) {
            motorValues = mecanumDrive(getMecX(), getMecY(), getMecRot());

            offsetTrim = dashboardCommunication.getOffset();
            scaleTrim = dashboardCommunication.getScale();

            motorValues = trimValues(motorValues, scaleTrim, offsetTrim);   //scale and offset the motors
            driveMotors(motorValues);   //set the motors to the scaled and offset values
            dashboardCommunication.drive();

            boolean[] driverStationIOState = null;
            try {
                for (int i = 1; i < 16; i++) {
                    driverStationIOState[i] = driverStationIO.getDigital(i);
                }

            } catch (EnhancedIOException ex) {
            }
            SmartDashboard.putString("Driver Station IO", driverStationIOState.toString());

            readWriteToFile();

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
        dashboardCommunication.visionProcessing(visionProcessing);

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
        fl.set(0);
        bl.set(0);
        br.set(0);
        fr.set(0);
    }

    private void readWriteToFile() {
        writeToFile = SmartDashboard.getBoolean("Test file output", false) && !writeToFile;

        if (writeToFile) {  //test writing to a file
            DataOutputStream file;
            FileConnection fc;
            try {
                fc = (FileConnection) Connector.open("file:///test.txt", Connector.WRITE);
                fc.create();
                file = fc.openDataOutputStream();
                file.writeUTF(SmartDashboard.getString("Data to write to file"));
                file.flush();
                file.close();
                fc.close();
            } catch (IOException ex) {
            }
        }

        readFromFile = SmartDashboard.getBoolean("Test file input", false) && !readFromFile;

        if (readFromFile) {   //test reaing from a file
            FileConnection fc;
            BufferedReader reader;
            try {
                fc = (FileConnection) Connector.open("file:///test.txt", Connector.READ);
                fc.create();
                reader = new BufferedReader((Reader) fc);
                String firstLine = reader.readLine();
                SmartDashboard.putString("First line of the file", firstLine);  //put the first line of the file on the SmartDashboard
                fc.close();
            } catch (IOException ex) {
            }
        }
    }
}
