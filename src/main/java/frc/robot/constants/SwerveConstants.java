
package frc.robot.constants;

public class SwerveConstants {

	public static double AngleError = 0;
	public static final double MaxSpeed = 2; // 6 meters per second desired top speed
    public static final double MaxAngularRate = 1 * Math.PI; // 3/4 of a rotation per second max angular velocity
    public static final double kAutonMoveSpeed = 1;
    public static final double kAlignmentOutput = 0.01;
		public static double kAlignmentTolerance = 1;
		public static double kStuckTolerance = -10;
		
		//this is for the horizontal yaw align to april tag control
		public static double kRotationPForVision = 0.085;
	public static double kRotationPForSwerve = 0.085;
		//0.07
		public static double kRotationIForVision = .000;
	public static double kRotationIForSwerve = .000;
		//.0001
		public static double kRotationDForSwerve = .00;
	public static double kRotationDForVision = .00;
		public static double speedpercentage = 0.5;//1.0
}
