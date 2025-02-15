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

  public static boolean kDebug = true;
  public static final String robotName = "Flipper";

  public static class SwerveSubsystem {

    public static final double MAX_SPEED = 12;
    public static final double LOOP_TIME = 0;
    public static final double ROBOT_MASS = 100 ;
    public static final Object CHASSIS = null;
    public static final double TURN_CONSTANT = 0;
    public static final double kTeleopDeadzone = 0.1;
    public static final double kAngleSpeedMultiplier = 1;

    /*
     * Motor ID's for the rev motors on the swerve Drive (in json file), Pheonix motors are set seperatly
     *-------------------------------------
     *  SparkMAX:
     * BACK LEFT ID-   1
     * FRONT RIGHT ID- 2
     * FRONT LEFT ID-  3
     * BACK RIGHT ID-  4
     * -------------------------------------
     * Falcon500:
     * BACK LEFT ID-   1
     * FRONT RIGHT ID- 2
     * FRONT LEFT ID-  3
     * BACK RIGHT ID-  4
     * -------------------------------------
     *  Enconder:
     * BACK LEFT ID-   5
     * FRONT RIGHT ID- 6
     * FRONT LEFT ID-  7
     * BACK RIGHT ID-  8
     */
  }

  // TODO: Create sub classes for each function of the Climber system
  public static class ClimberSubsystem {
    //Can ID's for climber are from 9 to 15
    public static final int kLockingBarMotorID = 9;
    public static final int kLatchingServoID = 0;
    public static final double kLockingMotorOffset = 0;
    public static final double kLockingBarMotorConversionFactor = 360;


      //Limits
    public static final double maxServoLimit = 0;
    public static final double minServoLimit = 0;

    //TODO; make sure these are right


    public static final double minBarLockLimit = 0.167*360;
    public static final double maxBarLockLimit = 0.425*360;

    public static final double unlockedBarPosition = 0;
    public static final double barLockedPosition = 0;

    public static final double servoLatchedAngle = 0;
    public static final double servoUnLatchedAngle = 0;

    public class PositionMotor {
      public static final int kMotorID = 10;
      public static final double kMaxAngle = 282.0; //0.037*360;
      public static final double kMinAngle = 132.0;//0.457*360;
      public static final double kConversionFactor = 360;
      public static final double kEncoderOffset = 0.5;    
      public static final double kDeployedAngle = 245.0; // 0.185*360;
      public static final double kHomeAngle = 135.0; //0.442*360;
    }
  }  

  // TODO: Create sub classes for each function of the Arm system
  public static class ArmSubsystem {
    //Can ID's for Arm are from 16 to 21
    public static final int kWristMotorID = 16;
    public static final int kPulleyMotorID = 17;
    public static final int kClawMotorID = 18;
    public static final int kElbowMotorID = 19;
    public static final double kWristMotorOffset = 0;
    public static final double kWristConversionFactor = 360;
    public static final double kPulleyMotorOffset = 0;
    public static final double kPulleyConversionFactor = 360;
    public static final double kReversedScrewMotorOffset = 0;
    public static final double kReversedScrewConversionFactor = 360;

      //set-to-point
    public static final double kL2Height = 0;
    public static final double kL3Height = 0;
    public static final double kL4Height = 0;
    public static final double kGroundHeight = 0;
    public static final double kHeightHumanStation = 0;

    public static final double kElbowPosGround = 0;
    public static final double kElbowPosL2 = 0;
    public static final double kElbowPosL3 = 0;
    public static final double kElbowPosL4 = 0;
    public static final double kElbowPosHumanStation = 0;

    public static final double kWristPosGround = 0;
    public static final double kWristPosL2 = 0;
    public static final double kWristPosL3 = 0;
    public static final double kWristPosL4 = 0;
    public static final double kWristPosHumanStation = 0;


      //Limits
    public static final double maxPulleyLimit = 0;
    public static final double minPulleyLimit = 0;
    public static final double minElbowLimit = 0;
    public static final double maxElbowLimit = 0;
    public static final double maxWristLimit = 0;
    public static final double minWristLimit = 0;
    public static final double closedClawPosition = 0;
    public static final double openedClawPosition = 0;

    public static final double armGoToTolerance = 0;
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
      public static final int deployClimberButton = 7;
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
