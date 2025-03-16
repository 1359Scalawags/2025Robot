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

  public static final boolean kDebug = false;
  public static final boolean kTuning = false;
  public static final String robotName = "Flipper";
  public static final double kRobotLoopTime = 0.02;

  public static class ArmSubsystem {
    public static final double kIntializeDelay = 1.0;

    public static class Positions {
      public static final ArmPosition kHome = new ArmPosition(1,200,215-94);
      public static final ArmPosition kHumanStation = new ArmPosition(4,200,215-94);
      public static final ArmPosition kGround = new ArmPosition(1,197,148-94);//correct
      public static final ArmPosition kLevel1 = new ArmPosition(1,295,208-94);//correct
      public static final ArmPosition kLevel2 = new ArmPosition(6.5,305,293-94);//correct
      public static final ArmPosition klevel3 = new ArmPosition(22.5,305,293-94);//correct
      public static final ArmPosition klevel4 = new ArmPosition(52,315,26-94);//correct
    }

    public static class Pulley{
      public static final int kMotorID = 17;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 0.13402;
      public static final double kMaxLimit = 52;
      public static final double kMinLimit = 0;
      public static final double kSlewRate = 5;
      public static final double kStageTwoPulleyPosition = 25; 
      public static final boolean kLimitSwitchPressedState = false;
      public static final int kHomeLimitSwitchID = 0;
      public static final double kLimitSwitchPosition = 1;
      public static final double kTolerance = 0.5;
      public static final double kHomingPositionIncrement = -5.0 * 0.020; // homing loop time is 20ms
      
      public static class PIDF{
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
        public static final double kP = 0.01;
        public static final double kI = 0.00001/8;
        public static final double kD = 0.01;
        public static final double kIZone = 5;
        public static final double kGravityFF = 0.35;
        public static final double kMINGravityFF = 0;
      }
    }

    public static class Wrist {
      public static final int kMotorID = 16;
      public static final double kMotorOffset = 0;
      public static final double kConversionFactor = 360;
      public static final double kMaxLimit = 286.0-94; // measure when elbow is horizontal
      public static final double kMinLimit = 147.0-94; // measure when elbow is horizontal
      public static final double kSlewRate = 30;
      public static final double kHorizontalAngle =232.0-94; 
      public static final double kTolerance = 2.0;
      public static final double kAccelerationRate = 25.0; //XXX: this is a guess
      
      public static class PIDF{
        public static final double kP = 0.01; //0.0075;
        public static final double kI = 0.00001; //0.00005;
        public static final double kD = 0.03; //0.006;
        public static final double kIZone = 5;
        public static final double kGravityFF = 0.25; //0.1;
        public static final double kMinGravityFF = 0;
      }
    }

    public static class Claw {
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
      public static final double kHomingPositionIncrement = 1.0 * 0.020; // homing loop time is 20ms
      public static final double kAccelerationRate = 5; //XXX: this is a guess
      
      public static class PIDF{
        public static final double kP = 0.075;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double kIZone = 2;
      }
    }
  }

  public static class Vision { 
    public static final String klimelightOne = "limelight-climber";
    public static final String klimelightTwo = "limelight-arm";
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
