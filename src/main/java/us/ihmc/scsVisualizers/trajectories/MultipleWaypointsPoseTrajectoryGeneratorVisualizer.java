package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class MultipleWaypointsPoseTrajectoryGeneratorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(46561L);
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final double trajectoryDuration = 5.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryDuration / dt / recordFrequency + 2);;

   private final MultipleWaypointsPoseTrajectoryGenerator trajectoryGenerator;

   private final YoFramePoint position = new YoFramePoint("position", worldFrame, registry);
   private final YoFrameVector velocity = new YoFrameVector("velocity", worldFrame, registry);
   private final YoFrameVector acceleration = new YoFrameVector("acceleration", worldFrame, registry);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public MultipleWaypointsPoseTrajectoryGeneratorVisualizer()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");
      
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.setGroundVisible(false);
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      String namePrefix = "traj";
      
      PoseReferenceFrame centerOfCircle = new PoseReferenceFrame("centerOfCircle", worldFrame);
      
      FramePose3D pose = new FramePose3D(worldFrame);
      pose.setOrientationYawPitchRoll(Math.PI / 4.0, Math.PI / 8.0, Math.PI / 16.0);
      pose.setPosition(0.4, 0.5, 0.2);
      centerOfCircle.setPoseAndUpdate(pose);
      
      pose.setToZero(centerOfCircle);
      FrameVector3D linearVelocity = new FrameVector3D(centerOfCircle);
      FrameVector3D angularVelocity = new FrameVector3D(centerOfCircle);
      FrameQuaternion orientation = new FrameQuaternion(centerOfCircle);
      
      int numberOfWaypoints = 200;
      double radius = 0.5;
      trajectoryGenerator = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix + "1", numberOfWaypoints, registry);
      
      
      YoPolynomial anglePolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 4, registry);
      anglePolynomial.setCubic(0.0, trajectoryDuration, 0.0, 0.0, Math.PI * 2.0, 0.0);
//      anglePolynomial.setLinear(0.0, trajectoryDuration, 0.0, Math.PI * 2.0);
      double t = 0.0;
      
      double timeBetweenWaypoints = trajectoryDuration / numberOfWaypoints;
      for(int i = 0; i < numberOfWaypoints;  i++)
      {
         pose.changeFrame(centerOfCircle);
         angularVelocity.changeFrame(centerOfCircle);
         linearVelocity.changeFrame(centerOfCircle);
         
         anglePolynomial.compute(t);

         double angle = anglePolynomial.getPosition();
         double angleDot = anglePolynomial.getVelocity();

         double x = radius * Math.cos(angle);
         double y = radius * Math.sin(angle);
         double z = 0.05 * Math.sin(angle * 10.0);
         
         double dx = radius * -Math.sin(angle) * angleDot;
         double dy = radius * Math.cos(angle) * angleDot;
         double dz = 0.05 * 10.0 * Math.cos(angle * 10.0) * angleDot;
         
         pose.setPosition(x, y, z);
         linearVelocity.set(dx, dy, dz);
         
         double yaw = Math.cos(angle) * 0.1;
         double pitch = -Math.cos(angle) * 0.15;
         double roll = Math.cos(angle) * 0.16;
         
         orientation.setYawPitchRoll(yaw, pitch, roll);
         pose.setOrientation(orientation);
         
         double yawRate = 0.1 * -Math.sin(angle) * angleDot;
         double pitchRate = 0.15 * Math.sin(angle) * angleDot;
         double rollRate = 0.16 * -Math.sin(angle) * angleDot;
         
         angularVelocity.changeFrame(centerOfCircle);
         RotationTools.computeAngularVelocityInBodyFrameFromYawPitchRollAnglesRate(yaw, pitch, roll, yawRate, pitchRate, rollRate, angularVelocity);
         
         pose.changeFrame(worldFrame);
         angularVelocity.changeFrame(worldFrame);
         linearVelocity.changeFrame(worldFrame);
         
         trajectoryGenerator.appendPoseWaypoint(t, pose, linearVelocity, angularVelocity);
         
         Graphics3DObject sphere = new Graphics3DObject();
         sphere.translate(pose.getPosition());
         sphere.addSphere(0.01,YoAppearance.Black());
         scs.addStaticLinkGraphics(sphere);
         
         t += timeBetweenWaypoints;
      }
      
      YoFramePose yoCurrentPose = new YoFramePose("yoCurrentPose", worldFrame, registry);
      YoGraphicCoordinateSystem currentPoseGraphic = new YoGraphicCoordinateSystem("currentPose", yoCurrentPose, 0.5);
      yoGraphicsListRegistry.registerYoGraphic("currentPoseGraphic", currentPoseGraphic);
      
      YoFrameVector yoLinearVelocity = new YoFrameVector("yoLinearVelocity", worldFrame, registry);
      YoFrameVector yoAngularVelocity = new YoFrameVector("yoAngularVelocity", worldFrame, registry);
      
      
      trajectoryGenerator.initialize();
      trajectoryGenerator.showVisualization();

      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      

      for (t = 0.0; t <= trajectoryDuration; t += dt)
      {
         robot.getYoTime().set(t);
         trajectoryGenerator.compute(t);
         trajectoryGenerator.getPosition(tempPoint);
         position.set(tempPoint);
         trajectoryGenerator.getVelocity(tempVector);
         velocity.set(tempVector);
         trajectoryGenerator.getAcceleration(tempVector);
         acceleration.set(tempVector);

         position.set(tempPoint);
         velocity.set(tempVector);
         acceleration.set(tempVector);
         
         trajectoryGenerator.getPose(pose);
         currentPoseGraphic.setPose(pose);
         
         trajectoryGenerator.getVelocity(linearVelocity);
         yoLinearVelocity.set(linearVelocity);
         trajectoryGenerator.getAngularVelocity(angularVelocity);
         yoAngularVelocity.set(angularVelocity);
         
         trajectoryGenerator.showVisualization();
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new MultipleWaypointsPoseTrajectoryGeneratorVisualizer();
   }
}