package org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.path;

import org.firstinspires.ftc.teamcode.Swerb506.utility.pathplanner.util.GeometryUtil;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.MathUtil;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Swerb506.utility.math.kinematics.ChassisSpeeds;

import java.util.ArrayList;
import java.util.List;

/** Trajectory created from a PathPlanner path */
public class PathPlannerTrajectory {
  private final List<State> states;

  /**
   * Generate a PathPlannerTrajectory
   *
   * @param path {@link PathPlannerPath} to generate the trajectory for
   * @param startingSpeeds Starting speeds of the robot when starting the trajectory
   */
  public PathPlannerTrajectory(PathPlannerPath path, ChassisSpeeds startingSpeeds) {
    this.states = generateStates(path, startingSpeeds);
  }

  private static int getNextRotationTargetIdx(PathPlannerPath path, int startingIndex) {
    int idx = path.numPoints() - 1;

    for (int i = startingIndex; i < path.numPoints() - 2; i++) {
      if (path.getPoint(i).holonomicRotation != null) {
        idx = i;
        break;
      }
    }

    return idx;
  }

  private static List<State> generateStates(PathPlannerPath path, ChassisSpeeds startingSpeeds) {
    List<State> states = new ArrayList<>();

    double startVel = Math.hypot(startingSpeeds.vxMetersPerSecond, startingSpeeds.vyMetersPerSecond);
    int nextRotationTargetIdx = getNextRotationTargetIdx(path, 0);

    for (int i = 0; i < path.numPoints(); i++) {
      State state = new State();

      PathConstraints constraints = path.getPoint(i).constraints;
      state.constraints = constraints;

      if (i > nextRotationTargetIdx) {
        nextRotationTargetIdx = getNextRotationTargetIdx(path, i);
      }

      state.targetHolonomicRotation = path.getPoint(nextRotationTargetIdx).holonomicRotation;

      state.positionMeters = path.getPoint(i).position;
      double curveRadius = path.getPoint(i).curveRadius;
      state.curvatureRadPerMeter = (Double.isFinite(curveRadius) && curveRadius != 0) ? 1.0 / curveRadius : 0.0;

      if (i == path.numPoints() - 1) {
        state.heading = states.get(states.size() - 1).heading;
        state.deltaPos = path.getPoint(i).distanceAlongPath - path.getPoint(i - 1).distanceAlongPath;
        state.velocityMps = path.getGoalEndState().getVelocity();
      } else if (i == 0) {
        state.heading = path.getPoint(i + 1).position.minus(state.positionMeters).getAngle();
        state.deltaPos = 0;
        state.velocityMps = startVel;
      } else {
        state.heading = path.getPoint(i + 1).position.minus(state.positionMeters).getAngle();
        state.deltaPos = path.getPoint(i + 1).distanceAlongPath - path.getPoint(i).distanceAlongPath;

        double v0 = states.get(states.size() - 1).velocityMps;
        double vMax = Math.sqrt(Math.abs(Math.pow(v0, 2) + (2 * constraints.getMaxAccelerationMpsSq() * state.deltaPos)));
        state.velocityMps = Math.min(vMax, path.getPoint(i).maxV);
      }

      states.add(state);
    }

    for (int i = states.size() - 2; i >= 1; i--) {
      PathConstraints constraints = states.get(i).constraints;

      double v0 = states.get(i + 1).velocityMps;
      double vMax = Math.sqrt(Math.abs(Math.pow(v0, 2) + (2 * constraints.getMaxAccelerationMpsSq() * states.get(i).deltaPos)));
      states.get(i).velocityMps = Math.min(vMax, states.get(i).velocityMps);
    }

    double time = 0;
    states.get(0).timeSeconds = 0;
    states.get(0).accelerationMpsSq = 0;
    states.get(0).headingAngularVelocityRps = startingSpeeds.omegaRadiansPerSecond;

    for (int i = 1; i < states.size(); i++) {
      double v0 = states.get(i - 1).velocityMps;
      double v = states.get(i).velocityMps;
      double dt = (2 * states.get(i).deltaPos) / (v + v0);

      time += dt;
      states.get(i).timeSeconds = time;

      double dv = v - v0;
      states.get(i).accelerationMpsSq = dv / dt;

      Rotation2d headingDelta = states.get(i).heading.minus(states.get(i - 1).heading);
      states.get(i).headingAngularVelocityRps = headingDelta.getRadians() / dt;
    }

    return states;
  }

  public State sample(double time) {
    if (time <= getInitialState().timeSeconds) return getInitialState();
    if (time >= getTotalTimeSeconds()) return getEndState();

    int low = 1;
    int high = getStates().size() - 1;

    while (low != high) {
      int mid = (low + high) / 2;
      if (getState(mid).timeSeconds < time) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }

    State sample = getState(low);
    State prevSample = getState(low - 1);

    if (Math.abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-3) return sample;

    return prevSample.interpolate(sample, (time - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds));
  }

  public List<State> getStates() {
    return states;
  }

  public double getTotalTimeSeconds() {
    return getEndState().timeSeconds;
  }

  public State getState(int index) {
    return getStates().get(index);
  }

  public State getInitialState() {
    return getState(0);
  }

  public Pose2d getInitialTargetHolonomicPose() {
    return getInitialState().getTargetHolonomicPose();
  }

  public Pose2d getInitialDifferentialPose() {
    return getInitialState().getDifferentialPose();
  }

  public State getEndState() {
    return getState(getStates().size() - 1);
  }

  /** A state along the trajectory */
  public static class State {
    public double timeSeconds = 0;
    public double velocityMps = 0;
    public double accelerationMpsSq = 0;
    public double headingAngularVelocityRps = 0;
    public Translation2d positionMeters = new Translation2d();
    public Rotation2d heading = new Rotation2d();
    public Rotation2d targetHolonomicRotation = new Rotation2d();
    public double curvatureRadPerMeter = 0;
    public double deltaPos = 0;
    public PathConstraints constraints = new PathConstraints();

    public Pose2d getTargetHolonomicPose() {
      return new Pose2d(positionMeters, targetHolonomicRotation);
    }

    public Pose2d getDifferentialPose() {
      return new Pose2d(positionMeters, heading);
    }

    public State interpolate(State other, double t) {
      State result = new State();
      result.timeSeconds = MathUtil.interpolate(this.timeSeconds, other.timeSeconds, t);
      result.velocityMps = MathUtil.interpolate(this.velocityMps, other.velocityMps, t);
      result.accelerationMpsSq = MathUtil.interpolate(this.accelerationMpsSq, other.accelerationMpsSq, t);
      result.headingAngularVelocityRps = MathUtil.interpolate(this.headingAngularVelocityRps, other.headingAngularVelocityRps, t);
      result.positionMeters = GeometryUtil.interpolate(this.positionMeters, other.positionMeters, t);
      result.heading = this.heading.interpolate(other.heading, t);
      result.targetHolonomicRotation = this.targetHolonomicRotation.interpolate(other.targetHolonomicRotation, t);
      result.curvatureRadPerMeter = MathUtil.interpolate(this.curvatureRadPerMeter, other.curvatureRadPerMeter, t);
      result.deltaPos = MathUtil.interpolate(this.deltaPos, other.deltaPos, t);
      result.constraints = this.constraints.interpolate(other.constraints, t);

      return result;
    }
  }
}
