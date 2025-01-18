package frc.robot.location;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import java.io.IOException;

public class FieldPositions {

  public static double FIELD_LENGTH = 16.541; // meters
  public static double FIELD_WIDTH = 8.211; // meters
  public static double TAG_OUTSIDE_FIELD = 0.04;

  private AprilTagFieldLayout layout;

  private static FieldPositions instance = null;

  public FieldPositions() {
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
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    }
  }

  public AprilTagFieldLayout getLayout() {
    return layout;
  }
}
