/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author schuyler
 */
public final class HelperFunctions {

    public static double maxAt1(double n) {
        return n < -1 ? -1 : (n > 1 ? 1 : n);
    }

    public static double abs(double value) {
        return value < 0 ? -value : value;
    }

    public static double deadZone(double value, double amount) {
        return (HelperFunctions.abs(value) < amount) ? 0 : value;
    }

    public static double deadZone(double value) {
        return deadZone(value, 0.2);
    }
    
}
