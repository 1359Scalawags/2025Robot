// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import static edu.wpi.first.units.Units.*;
import frc.robot.extensions.PidConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean kDebug = true;
  public static final boolean kTuning = false;
  
  public static final String robotName = "Flipper";
  public static final double kSimulationPidScalar = 2.5;
  public static final double kRobotLoopTime = Robot.isSimulation() ? 0.01 : 0.02;
  
  public static class TestElevator{
    public static final boolean kEnabled = true;
    public static final int kMotorID = 300;
    public static final double kMinHeightMeters = 0.5;
    public static final double kMaxHeightMeters = 2.5;
    public static final double kGearRatio = 64.0/1.0;
    public static final double kSpindleRadiusMeters = 0.0127;
    public static final double kMotorRotationsPerMeter = 1.0 /(2 * Math.PI * kSpindleRadiusMeters) * kGearRatio;
    public static final double kCarriageMassKg = Pound.of(2).in(Kilogram);
    public static final double kSpeedMetersPerSecond = 0.2;
    public static final PidConstants pid = new PidConstants(1, 0.001, 0.0, 0.0, 0.4);
  }

  public static class TestArm {
    public static final boolean kEnabled = true;
    public static final int kMotorID = 301;
    public static final double kMinAngleDegrees = -75;
    public static final double kMaxAngleDegrees = 60;   
    public static final double kGearRatio = 64.0/1.0;
    public static final double kArmLengthMeters = 0.5;
    public static final double kArmMassKg = 1.5;
    public static final double kMomentInertiaKgMM = SingleJointedArmSim.estimateMOI(kArmLengthMeters, kArmMassKg);
    public static final PidConstants pid = new PidConstants(0.025, 0.01, 0.0, 0.0, 0.2);
    public static final double kSpeedDegressPerSecond = 30;
  }

  public static class SwerveSubsystem {
    public static final boolean kEnabled = false;
    public static final double MAX_SPEED = 12;
    public static final double LOOP_TIME = 0;
    public static final double ROBOT_MASS = 100;
    public static final Object CHASSIS = null;
    public static final double TURN_CONSTANT = 0;
    public static final double kTeleopDeadzone = 0.1;
    public static final double kAngleSpeedMultiplier = 3.0/4.0;


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
     * FRONT RIGHT ID- 
     * FRONT LEFT ID-  7
     * BACK RIGHT ID-  8
     */
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

    public static class TestJoystick {
      public static final int kPort = 2;
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
