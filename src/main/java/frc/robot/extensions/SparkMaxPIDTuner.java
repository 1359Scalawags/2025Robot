package frc.robot.extensions;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;

public class SparkMaxPIDTuner {
    private ShuffleboardTab tab;
    private SparkClosedLoopController controller;
    private SparkMaxConfig config;
    private PIDController tuner;

    // public SparkMaxPIDTuner(String tabName, String widgetName, int column, SparkClosedLoopController controller) {
    //     this.controller = controller;
    //     this.tuner = new PIDController(this.controller.getP(), this.controller.getI(), this.controller.getD());
    //     this.tab = Shuffleboard.getTab(tabName);
    //     this.tab.add(widgetName, tuner).withPosition(column, 0).withSize(2, 2);
    //     this.tab.add("Apply " + widgetName + " values", new ApplyValues(this)).withPosition(column, 2).withSize(2, 1);
    // }

    // public void applyTunerValues() {
    //     config = new SparkMaxConfig();
    //     config.closedLoop.p(tuner.getP()).i(tuner.getI()).d(tuner.getD());
    //     controller.confi
    // }

    // public void resetTunerValues() {
    //     tuner.setP(controller.getP());
    //     tuner.setI(controller.getI());
    //     tuner.setD(controller.getD());
    // }


    public class ApplyValues extends Command {
        SparkMaxPIDTuner tuner;
    
        public ApplyValues(SparkMaxPIDTuner tuner) {
            this.tuner = tuner;
        }
    
        
        @Override
        public void initialize() {
            
        }
    
        // @Override
        // public void execute() {
        //     tuner.applyTunerValues();
        // }
    
        @Override
        public boolean isFinished() {
            return true;
        }
    
        @Override
        public void end(boolean interrupted) {
            
        }
    }



}
