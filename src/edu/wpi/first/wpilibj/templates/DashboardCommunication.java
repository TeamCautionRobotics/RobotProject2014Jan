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

}
