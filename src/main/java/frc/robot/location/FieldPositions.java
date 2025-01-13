package frc.robot.location;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.util.Arrays;

import bearlib.fms.AllianceColor;

public class FieldPositions{
    
    public static double FIELD_LENGTH = 17.55; // meters
    public static double FIELD_WIDTH = 8.05; // meters
    
    private AprilTagFieldLayout layout;
    private static FieldPositions instance = null;

    public FieldPositions(){
        initializeLayout();
    }

    public static FieldPositions getInstance() {
        if (instance == null) {
          instance = new FieldPositions();
        }
        return instance;
      }
    
    private void initializeLayout() {
        try {
          layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
          layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
          DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
        }
      }

    public AprilTagFieldLayout getLayout() {
        return layout;
    }









    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private static double DIST_TO_PERIM = inchesToMeters(130);
    private static double WIDTH_REEF = inchesToMeters(93.5);

    private static double REEF_SIDE_LENG = Math.sqrt(Math.pow(WIDTH_REEF, 2)/2);
    private static double ANG_BETW_SIDE = Math.toRadians(120);

    /**
     * Enum for the reef reference points.
     * Points follow a counter-clockwise order 
     * starting with the lowest and furthest left on the blue side.
     * Starting with the highest and furthest right on the red side.
     * this way the points will function as way points on both the red and blue sides
     */

     // since this class will only need to be constructed 
    public enum ReefReferencePoint {
        REF_POINT_ONE(new double[]{
            DIST_TO_PERIM,
            (FIELD_WIDTH / 2) - (REEF_SIDE_LENG / 2)
        }),
    
        REF_POINT_TWO(new double[]{
            REF_POINT_ONE.getRefCoordinates()[0] + Math.sin(ANG_BETW_SIDE) * REEF_SIDE_LENG,
            REF_POINT_ONE.getRefCoordinates()[1] + Math.cos(ANG_BETW_SIDE) * REEF_SIDE_LENG
        }),
    
        REF_POINT_THREE(new double[]{
            REF_POINT_TWO.getRefCoordinates()[0] + Math.sin(ANG_BETW_SIDE) * REEF_SIDE_LENG,
            REF_POINT_ONE.getRefCoordinates()[1]
        }),
    
        REF_POINT_FOUR(new double[]{
            REF_POINT_THREE.getRefCoordinates()[0],
            REF_POINT_ONE.getRefCoordinates()[1] + REEF_SIDE_LENG
        }),
    
        REF_POINT_FIVE(new double[]{
            REF_POINT_FOUR.getRefCoordinates()[0] - Math.sin(ANG_BETW_SIDE) * REEF_SIDE_LENG,
            REF_POINT_FOUR.getRefCoordinates()[1] - Math.cos(ANG_BETW_SIDE) * REEF_SIDE_LENG
        }),
    
        REF_POINT_SIX(new double[]{
            REF_POINT_ONE.getRefCoordinates()[0],
            REF_POINT_ONE.getRefCoordinates()[1] + REEF_SIDE_LENG
        });
    
        private final double[] coordinates;
    
        ReefReferencePoint(double[] coordinates) {
            this.coordinates = coordinates;
        }
    
        private double[] getRefCoordinates() {
            return coordinates;
        }
        public Pose2d getPose(){
      
            if(AllianceColor.getAlliance()==Alliance.Red){
                return new Pose2d(FIELD_LENGTH-coordinates[0],FIELD_WIDTH-coordinates[1], new Rotation2d());
        }
            return new Pose2d(coordinates[0],coordinates[1], new Rotation2d());
        }
    }
}
