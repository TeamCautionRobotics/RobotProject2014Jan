/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Caution
 */
class CatapultThread extends Thread{
    private int rot;
    private RobotMain robot;

    public CatapultThread(int rot, RobotMain robot) {
        this.rot = rot;
        this.robot = robot;
    }
    public void run(){
        if(rot == RobotMain.ROTARY_LOW_GOAL || rot == RobotMain.ROTARY_TRUSS){
            Timer.delay(.1);
            robot.catapult.set(Relay.Value.kForward);
        }
        if(rot == RobotMain.ROTARY_LOW_GOAL){
            Timer.delay(.2);
            robot.catapult.set(Relay.Value.kForward);
        }
    }
    
}
