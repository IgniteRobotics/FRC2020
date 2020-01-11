/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int kLeftMasterPort = 1;
    public static final int kLeftFollowerPort = 2;
    public static final int kLeftFollowerPort2 = 3;
    public static final int kRightMasterPort = 4;
    public static final int kRightFollowerPort = 5;
    public static final int kRightFollowerPort2 = 6;

    public static final int kDriveControllerPort = 0;
    public static final int kManipControllerPort = 1;

    public static final double kDriveDeadband = 0.03;

    public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int BUTTON_LEFT_BUMPER = 5;
	public static final int BUTTON_RIGHT_BUMPER = 6;
	public static final int BUTTON_BACK = 7;
	public static final int BUTTON_START = 8;
	public static final int BUTTON_LEFT_STICK = 9;
	public static final int BUTTON_RIGHT_STICK = 10;

	public static final int AXIS_LEFT_STICK_X = 0;
	public static final int AXIS_LEFT_STICK_Y = 1;
	public static final int AXIS_LEFT_TRIGGER = 2;
	public static final int AXIS_RIGHT_TRIGGER = 3;
	public static final int AXIS_RIGHT_STICK_X = 4;
	public static final int AXIS_RIGHT_STICK_Y = 5;

	public static final int BUTTON_DPAD_UP = 0;
	public static final int BUTTON_DPAD_LEFT = 270;
	public static final int BUTTON_DPAD_RIGHT = 90;
	public static final int BUTTON_DPAD_DOWN = 180;
}
