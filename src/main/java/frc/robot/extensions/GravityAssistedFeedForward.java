package frc.robot.extensions;

public class GravityAssistedFeedForward {
    
    private double m_minimum;
    private double m_gravity;
    private double m_offsetAngle;
    private double m_direction;

    /**
     * @param gravityFF Amount of feedforward needed to offset gravity.
     * @param offsetAngle Angle of the mechanism when horisontal (<em>and positive change in angle results in upward motion</em>).
     */
    public GravityAssistedFeedForward(double gravityFF, double offsetAngle) {
        this(0.0, gravityFF, offsetAngle, false);
    }

    /**
     * @param minimumFF Minimum amount of gravity assist <em>regardless of angle</em>.
     * @param gravityFF Amount of additional feedforward needed to offset gravity.
     * @param offsetAngle Angle of the mechanism when horisontal (<em>and positive change in angle results in upward motion</em>).
     */
    public GravityAssistedFeedForward(double minimumFF, double gravityFF, double offsetAngle) {
        this(minimumFF, gravityFF, offsetAngle, false);
    }
 
    /**
     * @param minimumFF Minimum amount of gravity assist regardless of angle. 
     * @param gravityFF Amount of additional feedforward needed to offset gravity.
     * @param offsetAngle Angle of the mechanism when horisontal (<em>and positive change in angle results in upward motion</em>).
     * @param reverse Set this to true if motor output and resulting angular change are opposites.
     */
    public GravityAssistedFeedForward(double minimumFF, double gravityFF, double offsetAngle, boolean reverse) {
        this.m_minimum = minimumFF;
        this.m_gravity = gravityFF;
        this.m_offsetAngle = offsetAngle;
        if(reverse) {
            m_direction = -1.0;
        } else {
            m_direction = 1.0;
        }
    }

    public void setMinimumFF(double value) {
        this.m_minimum = value;
    }

    public double getMinimumFF() {
        return this.m_minimum;
    }

    public void setGravityFF(double value) {
        this.m_gravity = value;
    }

    double getGravityFF() {
        return this.m_gravity;
    }

    /**
     * Calculate the gravity compensating feed forward value for the current angle.
     * This method must be updated periodically.
     * @param currentAngle The current angle of the mechanism as reported by encoders.
     * @return The calculated feed forward value.
     */
    public double calculate(double currentAngle) {
        double relativeRadians = Math.toRadians(currentAngle - m_offsetAngle);
        double cosineValue = Math.cos(relativeRadians);
        return (Math.copySign(m_minimum, cosineValue) +  m_gravity * cosineValue) * m_direction;
    }

    @Override
    public String toString() {
        return "GravityAssistFeedForward[min: " + m_minimum + ", gravity: " + m_gravity + "]";
    }
}
