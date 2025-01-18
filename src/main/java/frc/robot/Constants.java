// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.util.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SwereveSubsystem {
    /*
     * Motor ID's for the rev motors on the swerve Drive (in json file), Pheonix motors are set seperatly
     *-------------------------------------
     *  SparkMAX:
     * FRONT LEFT-  1
     * FRONT RIGHT- 2
     * BACK LEFT-   3
     * BACK RIGHT-  4
     * -------------------------------------
     * Falcon500:
     * FRONT LEFT-  1
     * FRONT RIGHT- 2
     * BACK LEFT-   3
     * BACK RIGHT-  4
     * -------------------------------------
     */



  }

  public static class ClimberSubsystem {
    public static final int kBabyLockingMotorID = 5;
    public static final int kPositionMotorID = 6;


  }  

  public static class ArmSubsystem {
    public static final int kWristMotorID = 7;
    public static final int kPullyMotorID = 8;
    public static final int kReversedScrewMotorID = 9;
    public static final int kArmMotorID = 10;


  }

  public static class Vision {

  }


  public static class OperatorConstants {

    public static class DriverJoysticks {

    }

    public static class AssistSubsystem {

    }

    public static final int kDriverControllerPort = 0;
  }
}
