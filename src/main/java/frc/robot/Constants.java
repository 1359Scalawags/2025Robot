// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.extensions.ArmPosition;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static boolean kDebug = false;
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

      //Limits 
      //TODO; make sure these are right


    public static class LatchServo {
      public static final int kServoID = 0;
      public static final double maxLimit = 0;
      public static final double minLimit = 0;
      public static final double latchedValue = 0.8;
      public static final double unLatchedValue = 0.2;
      
    }

    public static class LockingBarMotor {
      public static final int kMotorID = 9;
      public static final double kEncoderOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMinLimit = 72;
      public static final double kMaxLimit = 155;
      public static final double kUnLockedPosition = 63.0;
      public static final double kLockedPosition = 150.0;
      public static final double kSlewRate = 15;
    }

    public static class PositionMotor {
      public static final int kMotorID = 10;
      public static final double kMaxAngle = 275.0;//0.037*360;
      public static final double kMinAngle = 150.0;//0.457*360;
      public static final double kConversionFactor = 360;
      public static final double kEncoderOffset = 0.5;    
      public static final double kDeployedAngle = 245.0; // 0.185*360;
      public static final double kHomeAngle = 160.0; //0.442*360;

      public static final double kMaxJoystickSpeed = 10;
      public static final double kSlewRate = 30.0;
    }
  }  

  // TODO: Create sub classes for each function of the Arm system
  public static class ArmSubsystem {
    //Can ID's for Arm are from 16 to 21

    public static class Positions {
      public static final ArmPosition kHome = new ArmPosition(0,0,0);
      public static final ArmPosition kHumanStation = new ArmPosition(0,0,0);
      public static final ArmPosition kGround = new ArmPosition(0,0,0);
      public static final ArmPosition kLevel1 = new ArmPosition(0,0,0);
      public static final ArmPosition kLevel2 = new ArmPosition(0,0,0);
      public static final ArmPosition klevel3 = new ArmPosition(0,0,0);
      public static final ArmPosition klevel4 = new ArmPosition(0,0,0);
    }

    public static class Pulley{
      public static final int kMotorID = 17;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 0;
      public static final double kMinLimit = 0;
      public static final double kSlewRate = 0;
      public static final boolean kLimitSwitchPressedState = false;
      public static final double kHomingVelocity = 0;
      public static final double kPositionTolerance = 1.5;
    }

    public static class Elbow {
      public static final int kMotorID = 19;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 0;
      public static final double kMinLimit = 0;
      public static final double kSlewRate = 0;
      public static final double kPositionTolerance = 1.5;
    }

    public static class Wrist {
      public static final int kMotorID = 16;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 0;
      public static final double kMinLimit = 0;
      public static final double kSlewRate = 0;
      public static final double kPositionTolerance = 1.5;
    }

    public static class Claw {
      public static final int kMotorID = 18;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 0;
      public static final double kMinLimit = 0;
      public static final double kSlewRate = 0;
      public static final double kPositionTolerance = 1.5;
    }

    public static final double armGoToTolerance = 0;
    public static final int kHomeLimitSwitchID = 0;
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
