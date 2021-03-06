// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int LEFT_FRONT = 0;
	public static final int RIGHT_FRONT = 1;
	public static final int LEFT_BACK = 2;
	public static final int RIGHT_BACK = 3;
	public static final double SPEED_FB = 0.5;
	public static final double SPEED_STRF = 0.5;
	public static final double SPEED_TURN = 0.3;
	public static final int DRIVER_JOY = 0;
	public static final int SHOOTER_ID = 4;

	public static final int kSlotIdx = 0;
	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;
	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;
	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut */
   public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
	public static final double SHOOTER_LIMIT = 0.5;
	public static final int INTAKE_ID = 5;
	public static final double INTAKE_SPEED = .99;
	public static final int CAM_RES_X = 640;
	public static final int CAM_RES_Y = 480;
	public static final double TICKS_PER_REV = 8192;
	public static final double CASTER_WHEEL_RADIUS = 4; // in
	public static final double GEAR_RATIO = 1;
	public static final int PIGEON_ID = 0;
	public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
	public static double FORWARD_OFFSET = 4;
    
}
