/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author schuyler
 */
class DashboardCommunication {

    private RobotMain robot;

    final int frontLeft = 0;
    final int backLeft = 1;
    final int backRight = 2;
    final int frontRight = 3;

    public DashboardCommunication() {

    }

    DashboardCommunication(RobotMain r) {
        this.robot = r;
    }

    void visionProcessing(VisionProcessing visionProcessing) {
        System.out.println("Target Hot" + visionProcessing.target.Hot);
        SmartDashboard.putBoolean("Target Hot", visionProcessing.target.Hot);
    }

    void drive() {
        System.out.println("Mecanum X" + robot.getMecX());
        System.out.println("Mecanum Y" + robot.getMecY());
        System.out.println("Mecanum Rotation" + robot.getMecRot());
        System.out.println("Front Left" + robot.fl.getSpeed());
        System.out.println("Front Right" + robot.fr.getSpeed());
        System.out.println("Back Left" + robot.bl.getSpeed());
        System.out.println("Back Right" + robot.br.getSpeed());

        SmartDashboard.putNumber("Mecanum X", robot.getMecX());   //put the different motor and joystick values on the dashboard for debugging
        SmartDashboard.putNumber("Mecanum Y", robot.getMecY());
        SmartDashboard.putNumber("Mecanum Rotation", robot.getMecRot());
        SmartDashboard.putNumber("Front Left", robot.fl.getSpeed());
        SmartDashboard.putNumber("Front Right", robot.fr.getSpeed());
        SmartDashboard.putNumber("Back Left", robot.bl.getSpeed());
        SmartDashboard.putNumber("Back Right", robot.br.getSpeed());
    }

    double[] getOffset() {
        double[] offsetTrim = null;
        offsetTrim[frontLeft] = SmartDashboard.getNumber("Front Left Offset", 0);  //get the offsets for the motors
        offsetTrim[frontRight] = SmartDashboard.getNumber("Front Right Offset", 0);
        offsetTrim[backLeft] = SmartDashboard.getNumber("Back Left Offset", 0);
        offsetTrim[backRight] = SmartDashboard.getNumber("Back Right Offset", 0);

        return offsetTrim;
    }

    double[] getScale() {
        double[] scaleTrim = null;
        scaleTrim[frontLeft] = SmartDashboard.getNumber("Front Left Scale", 1);    //get the scaling for the motors
        scaleTrim[frontRight] = SmartDashboard.getNumber("Front Right Scale", 1);
        scaleTrim[backLeft] = SmartDashboard.getNumber("Back Left Scale", 1);
        scaleTrim[backRight] = SmartDashboard.getNumber("Back Right Scale", 1);

        return scaleTrim;
    }
}
