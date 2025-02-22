// Copyright (c) 2025 FRC 4068
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.field;

import edu.wpi.first.math.geometry.*;
import frc.robot.utils.AllianceFlipUtil;
import lombok.Getter;

/** */
@Getter
public class ReefAutoAlignZone {
  public Translation2d center;
  public Translation2d point1;
  public Translation2d point2;
  public Rotation2d angle;
  public Pose2d scorePose;

  public ReefAutoAlignZone(
      Translation2d center,
      Translation2d point1,
      Translation2d point2,
      Rotation2d angle,
      Pose2d scorePose) {
    this.center = center;
    this.point1 = point1;
    this.point2 = point2;
    this.angle = angle;
    this.scorePose = scorePose;
  }

  private double area(Translation2d a, Translation2d b, Translation2d c) {
    return Math.abs(
            a.getX() * (b.getY() - c.getY())
                + b.getX() * (c.getY() - a.getY())
                + c.getX() * (a.getY() - b.getY()))
        / 2.0;
  }

  public boolean containsPoint(Translation2d point) {
    double originalArea = area(center, point1, point2);
    double area1 = area(point, AllianceFlipUtil.apply(center), AllianceFlipUtil.apply(point1));
    double area2 = area(point, AllianceFlipUtil.apply(point1), AllianceFlipUtil.apply(point2));
    double area3 = area(point, AllianceFlipUtil.apply(center), AllianceFlipUtil.apply(point2));

    //    return (area1 + area2 + area3 - originalArea);
    return Math.abs(area1 + area2 + area3 - originalArea) < .01;
  }
}
