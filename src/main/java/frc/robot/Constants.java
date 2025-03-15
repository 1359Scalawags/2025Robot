// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
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
  public static boolean kTuning = false;
  public static final String robotName = "Flipper";
  public static final double kRobotLoopTime = 0.02;

  public static class SwerveSubsystem {

    public static final double MAX_SPEED = 12;
    public static final double LOOP_TIME = 0;
    public static final double ROBOT_MASS = 100 ;
    public static final Object CHASSIS = null;
    public static final double TURN_CONSTANT = 0;
    public static final double kTeleopDeadzone = 0.1;
    public static final double kAngleSpeedMultiplier = 3/4;

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

  public static class ClimberSubsystem {
    //Can ID's for climber are from 9 to 15

      //Limits 
      //TODO; make sure these are right
    public static final double kIntializeDelay = 0.5;

    public static class LatchServo {
      public static final int kServoID = 0;
      public static final double maxLimit = 1;
      public static final double minLimit = 0;
      public static final double latchedValue = 0.8;
      public static final double unLatchedValue = 0.2;
      public static final double kNaxActuateTime = 5.0; //max time to run the servo
      
    }

    public static class LockingBarMotor {
      public static final int kMotorID = 9;
      public static final double kEncoderOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMinLimit = 72;
      public static final double kMaxLimit = 155;
      public static final double kUnLockedPosition = 72.0;
      public static final double kLockedPosition = 155.0;
      public static final double kSlewRate = 90;
    public static class PIDF{
      // 0.01 , 0.000001, 0.007
      public static final double kP = 0.01;
      public static final double kI = 0.000001;
      public static final double kD = 0.007;
      public static final double kIZone = 5;
      }
    }

    public static class PositionMotor {
      public static final int kMotorID = 10;
      public static final double kMaxAngle = 275.0;//0.037*360;
      public static final double kMinAngle = 143.0;//0.457*360;
      public static final double kConversionFactor = 360;
      public static final double kEncoderOffset = 0.5;    
      public static final double kDeployedAngle = 245.0; // 0.185*360;
      public static final double kHomeAngle = 150.0; //0.442*360;
      public static final double kLockingPosition = 170.0;
      public static final double kLockedPosition = 177.5;
      public static final double kMaxJoystickSpeed = 10;
      public static final double kSlewRate = 45.0;
    public static class PIDF{
       // 0.05, 0.0000001, 0.08
      public static final double kP = 0.05;
      public static final double kI = 0.0000001;
      public static final double kD = 0.08;
      public static final double kIZone = 30;
      }
    }
  }  

  public static class ArmSubsystem {
    //Can ID's for Arm are from 16 to 21
    public static final double kIntializeDelay = 1.0;

    public static class Positions {
      public static final ArmPosition kHome = new ArmPosition(1,200,215);
      public static final ArmPosition kHumanStation = new ArmPosition(4,200,215);
      public static final ArmPosition kGround = new ArmPosition(1,197,148);//correct
      public static final ArmPosition kLevel1 = new ArmPosition(1,295,208);//correct
      public static final ArmPosition kLevel2 = new ArmPosition(6.5,305,293);//correct
      public static final ArmPosition klevel3 = new ArmPosition(22.5,305,293);//correct
      public static final ArmPosition klevel4 = new ArmPosition(52,315,260);//correct

      // public static final ArmPosition kHome = new ArmPosition(0,232,200);
      // public static final ArmPosition kHumanStation = new ArmPosition(0,232,210);
      // public static final ArmPosition kGround = new ArmPosition(0,232,220);//correct
      // public static final ArmPosition kLevel1 = new ArmPosition(0,232,232);//correct
      // public static final ArmPosition kLevel2 = new ArmPosition(0,232,240);//correct
      // public static final ArmPosition klevel3 = new ArmPosition(0,232,250);//correct
      // public static final ArmPosition klevel4 = new ArmPosition(0,232,260);//correct
    }

    public static class Pulley{
      public static final int kMotorID = 17;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 0.13402;
      public static final double kMaxLimit = 52;
      public static final double kMinLimit = 0;
      public static final double kSlewRate = 5;
      //public static final double kHomingVelocity = -2;
      public static final double kStageTwoPulleyPosition = 25; 
      public static final boolean kLimitSwitchPressedState = false;
      public static final int kHomeLimitSwitchID = 0;
      public static final double kLimitSwitchPosition = 1;
      public static final double kTolerance = 0.5;
      public static final double kHomingPositionIncrement = -5.0 * 0.020; // homing loop time is 20ms
    public static class PIDF{
       //0.07f, 0.00003f, 0.07f
      public static final double kP = 0.07;
      public static final double kI = 0.00003;
      public static final double kD = 0.07;
      public static final double kIZone = 0;//5;
      public static final double kStageOneFF = 0;//0.15;
      public static final double kStageTwoFF = 0;//0.2;
      }
    }

    public static class Elbow {
      public static final int kMotorID = 19;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 330;
      public static final double kMinLimit = 195;
      public static final double kSlewRate = 45;
      public static final double kHorizontalAngle = 232.0;
      public static final double kTolerance = 1.0;
      public static final double kAccelerationRate = 25.0;
      public static class PIDF{
          //0.025, 0.00001, 0.07
        public static final double kP = 0.01;
        public static final double kI = 0.00001/8;
        public static final double kD = 0.01;
        public static final double kIZone = 5;
        public static final double kGravityFF = 0.5;
        public static final double kMINGravityFF = 0;
        }
    }

    public static class Wrist {
      public static final int kMotorID = 16;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 286.0; // measure when elbow is horizontal
      public static final double kMinLimit = 147.0; // measure when elbow is horizontal
      public static final double kSlewRate = 30;
      public static final double kHorizontalAngle =232.0; //XXX: This is just a guess...must fix
      public static final double kTolerance = 2.0;
    public static class PIDF{
      // (0.006, 0.0000006, 0.006)
      public static final double kP = 0.0; //0.0075;
      public static final double kI = 0.0; //0.00005;
      public static final double kD = 0.0; //0.006;
      public static final double kIZone = 5;
      public static final double kGravityFF = 0.0; //0.1;
      public static final double kMinGravityFF = 0;
      }
    }

    public static class Claw {
      // posotive motor value closes claw, negative opens
      public static final int kMotorID = 18;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 0; //good
      public static final double kMinLimit = -17.38;
      public static final double kSlewRate = 20;
      public static final double kCloseClaw = 0;//good
      public static final double kOpenClaw = -17.38;//good
      public static final boolean kLimitSwitchPressedState = false;
      public static final int kLimitSwitchID = 1;
      //public static final double kHomingVelocity = 300;
      public static final double kHomingPositionIncrement = 1.0 * 0.020; // homing loop time is 20ms
      
    public static class PIDF{
          //0.05, 0.0001, 0.03
      public static final double kP = 0.05;
      public static final double kI = 0.0001;
      public static final double kD = 0.03;
      public static final double kIZone = 2;
      }
    }
  }

  public static class Vision { 
    public static final String klimelightOne = "limelight2+";
    public static final String klimelightTwo = "limelight3G";
  }


  public static class Operator {

    public static class DriverJoystick {
    public static final int kPort = 0;
    public static final int driveForwardButton = 12;
    public static final int driveRightButton = 13;
    public static final int rotateCCWButton = 14;
    }

    public static class AssistJoystick {
      public static final int kPort = 1;
      public static final int deployClimberButton = 7;
    }
  }

  public static class Testing {
    public static final double kSwerveTranslateSpeed = 0.3;
    public static final double kSwerveRotateSpeed = 0.5;
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
