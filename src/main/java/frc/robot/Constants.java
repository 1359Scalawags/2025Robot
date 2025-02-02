// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFields;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static boolean kDebug = true;
  public static final String robotName = "Siren";

  public static class SwerveSubsystem {

    public static final double MAX_SPEED = 0;
    public static final double LOOP_TIME = 0;
    public static final double ROBOT_MASS = 0;
    public static final Object CHASSIS = null;
    public static final double TURN_CONSTANT = 0;

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
    

     //TODO: what is this for?
     public SwerveSubsystem(File file) {
      //TODO Auto-generated constructor stub
    }

    

  }

  public static class ClimberSubsystem {
    public static final int kLockingMotorID = 5;
    public static final int kPositionMotorID = 6;
    public static final float kPositionConversionFactor = 1.0f;
    public static final double kPositionEncoderOffset = 0;
    public static final int kLatchingServoID = 0;
    public static final double kLockingMotorOffset = 0;
    public static final double kLockingMotorConversionFactor = 0;

    public static final double maxServoLimit = 0;
    public static final double minServoLimit = 0;

    public static final double maxClimberAngle = 0;
    public static final double minClimberAngle = 0;

    public static final double minLockLimit = 0;
    public static final double maxLockLimit = 0;
    public static final double unlockedPosition = 0;
    public static final double lockedPosition = 0;
    public static final double extendedClimberAngle = 0;
    public static final double retractedClimberAngle = 0;

  }  

  public static class ArmSubsystem {
    public static final int kWristMotorID = 7;
    public static final int kPulleyMotorID = 8;
    public static final int kReversedScrewMotorID = 9;
    public static final int kArmMotorID = 10;
    public static final double kWristMotorOffset = 0;
    public static final double kWristConversionFactor = 0;
    public static final double kPulleyMotorOffset = 0;
    public static final double kPulleyConversionFactor = 0;
    public static final double kReversedScrewMotorOffset = 0;
    public static final double kReversedScrewConversionFactor = 0;
    public static final double maxPulleyLimit = 0;
    public static final double minPulleyLimit = 0;
    public static final double minElbowLimit = 0;
    public static final double maxElbowLimit = 0;
    public static final double maxWristLimit = 0;
    public static final double minWristLimit = 0;


  }

  public static class Vision { 
    public static final String klimelightOne = "limelight2+";
    public static final String klimelightTwo = "limelight3G";
  }


  public static class Operator {

    public static class DriverJoystick {
      public static final int kPort = 0;
    }

    public static class AssistJoystick {
      public static final int kPort = 1;
      public static final BooleanSupplier climberExtendButton = null;
    }
  }


/*    
          ................................................................................          
          ................................................................................          
          ................................................................................          
          ..................................:%@@@@@@=..-..................................          
          ...............................:@%@@@@@@@@@@@@@@@...............................          
          ..............................:=@@@@@@@@@@@@@@@@@@#.............................          
          .............................+@@@#=@@@@@@@@@@@@@@@@.............................          
          ............................:@#@@@@@@#++@@@@@@@@@@@@............................          
          ............................+@@@@*.....%@%.........%:...........................          
          .............................@+@@......@@@.......++@............................          
          ..............................+@@#==@@@+.#@-...+@@:.............................          
          .............................+@@@@@@@@+...#@@@@@@@%.............................          
          .................................#.@@@@@@@@@@+.#=...............................          
          ..................=-.............@:.%@@@@@@@@.-@=...............................          
          ................-@%=%%-..........-@@-..+--...@@.............%@@.................          
          ...............@@@*-=++@@%.........=@@@@@@@@@=...........-@@@%@@................          
          .......................-#%*=%@#-....-@@@%#@%=...:++.-#@@**+.....................          
          ...............................*@@@@@+....=#*-+-:@%.............................          
          ....................................*@@@@@%:....................................          
          ..........................=#@@@@@@@=.....%@#=@@@@@+.............................          
          ...............@%%+.:.=@@#+@%-.................-+==@@@@#-=##=-+@@@..............          
          ................++@@@@@#-...........--...................:@@@@@+................          
          .................*@@#.......................................#@@@................          
          ..................=.............................................................          
          ................................................................................          
          ................................................................................          
          ................................................................................            
   */
}
