package us.ihmc.scsVisualizers.kinematicsBasedStateEstimation;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ClippedSpeedOffsetErrorInterpolator;
import us.ihmc.commons.thread.ThreadTools;

public class ClippedSpeedOffsetErrorInterpolatorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyTransform referenceFrameToBeCorrectedTransform = new RigidBodyTransform();
   private final Vector3D referenceFrameToBeCorrectedTransform_Translation = new Vector3D();
   private final Quaternion referenceFrameToBeCorrectedTransform_Rotation = new Quaternion();
   
   private final ReferenceFrame referenceFrameToBeCorrected = new ReferenceFrame("referenceFrameToBeCorrected", worldFrame)
   {
      
      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.set(referenceFrameToBeCorrectedTransform);
         
      }
   };
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double dt = 0.001;
   private final int recordFrequency = 5;
   int maxTimeStamp = 6000;
   private final int bufferSize = maxTimeStamp;
   
   private final YoDouble alphaFilterBreakFrequency = new YoDouble("alphaFilterBreakFrequency", registry);
   
   private final YoLong timeStamp = new YoLong("timeStamp", registry);
   
   int numberOfWaypoints = 2; 
   TimeStampedTransformBuffer referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame = new TimeStampedTransformBuffer(numberOfWaypoints);
   
   public ClippedSpeedOffsetErrorInterpolatorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      scs.setGroundVisible(false);
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(linkGraphics);
      
      Vector3D referenceFrameToBeCorrectedStartLocationInWorld = new Vector3D(-0.5, 0.0, 0.0);
      Vector3D referenceFrameToBeCorrectedEndLocationInWorld = new Vector3D(1.0, 1.0, 1.0);
      Quaternion referenceFrameToBeCorrectedStartOrientationInWorld = new Quaternion();
      referenceFrameToBeCorrectedStartOrientationInWorld.setYawPitchRoll(Math.toRadians(0.0), 1.0, -1.0);
      Quaternion referenceFrameToBeCorrectedEndOrientationInWorld = new Quaternion();
      referenceFrameToBeCorrectedEndOrientationInWorld.setYawPitchRoll(Math.toRadians(0.0), -1.0, 1.0);
      
//      generateKnownTranslationOnlyReferenceFrameToBeCorrectedWaypoints(referenceFrameToBeCorrectedStartLocationInWorld, referenceFrameToBeCorrectedEndLocationInWorld);
//      generateKnownRotationOnlyReferenceFrameToBeCorrectedWaypoints(referenceFrameToBeCorrectedStartOrientationInWorld, referenceFrameToBeCorrectedEndOrientationInWorld);
      generateKnownReferenceFrameToBeCorrectedWaypoints(referenceFrameToBeCorrectedStartLocationInWorld, referenceFrameToBeCorrectedEndLocationInWorld, referenceFrameToBeCorrectedStartOrientationInWorld, referenceFrameToBeCorrectedEndOrientationInWorld);
//      generateRandomReferenceFrameToBeCorrectedWaypoints();

      TimeStampedTransform3D timeStampedTransform = new TimeStampedTransform3D();
      FramePose3D referenceFrameToBeCorrectedPose = new FramePose3D(referenceFrameToBeCorrected);
      YoFramePoseUsingYawPitchRoll yoReferenceFrameToBeCorrectedPose = new YoFramePoseUsingYawPitchRoll("referenceFrameToBeCorrected", worldFrame, registry);
      YoGraphicCoordinateSystem referenceFrameToBeCorrectedPoseGraphic = new YoGraphicCoordinateSystem("referenceFrameToBeCorrectedPoseGraphic", yoReferenceFrameToBeCorrectedPose, 1.0 ,YoAppearance.Black());
      yoGraphicsListRegistry.registerYoGraphic("referenceFrameTobeCorrected", referenceFrameToBeCorrectedPoseGraphic);
      
      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, timeStampedTransform );
      referenceFrameToBeCorrectedTransform.set(timeStampedTransform.getTransform3D());
      referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
      referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
      referenceFrameToBeCorrected.update();
      
      referenceFrameToBeCorrectedPose.setToZero(referenceFrameToBeCorrected);
      referenceFrameToBeCorrectedPose.changeFrame(worldFrame);
      yoReferenceFrameToBeCorrectedPose.set(referenceFrameToBeCorrectedPose);

      Vector3D startPose_Translation = new Vector3D(0.0, 0.0, 0.0);
      Quaternion startPose_Rotation = new Quaternion();
      startPose_Rotation.setYawPitchRoll(0.0, 0.0, 0.0);
      RigidBodyTransform startPoseTransform = new RigidBodyTransform();

      startPoseTransform.setIdentity();
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), referenceFrameToBeCorrectedTransform_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), startPose_Translation));
      startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));

      FramePose3D startPose = new FramePose3D(worldFrame);
      startPose.set(startPoseTransform);
      YoFramePoseUsingYawPitchRoll yoStartPose = new YoFramePoseUsingYawPitchRoll("startPose", worldFrame, registry);
      yoStartPose.set(startPose);
      YoGraphicCoordinateSystem startPoseGraphic = new YoGraphicCoordinateSystem("startPoseGraph", yoStartPose, 0.7, YoAppearance.Yellow());
      yoGraphicsListRegistry.registerYoGraphic("inputs", startPoseGraphic);

      RigidBodyTransform goalPoseTransform = new RigidBodyTransform();
      Vector3D goalPose_Translation = new Vector3D(0.5, 0.0, 0.0);
      Quaternion goalPose_Rotation = new Quaternion();
      goalPose_Rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), 0.0);
      
      goalPoseTransform.setIdentity();
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), referenceFrameToBeCorrectedTransform_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), goalPose_Translation));
      goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
      goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));

      FramePose3D goalPose = new FramePose3D(worldFrame);
      goalPose.set(goalPoseTransform);
      YoFramePoseUsingYawPitchRoll yoGoalPose = new YoFramePoseUsingYawPitchRoll("goalPose", worldFrame, registry);
      yoGoalPose.set(goalPose);
      YoGraphicCoordinateSystem goalPoseGraphic = new YoGraphicCoordinateSystem("goalPoseGraph", yoGoalPose, 0.7, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("inputs", goalPoseGraphic);

      FramePose3D interpolatedPose = new FramePose3D(startPose);
      YoFramePoseUsingYawPitchRoll yoInterpolatedPose = new YoFramePoseUsingYawPitchRoll("interpolatedPose", worldFrame, registry);
      yoInterpolatedPose.set(interpolatedPose);
      YoGraphicCoordinateSystem interpolatedPoseGraphic = new YoGraphicCoordinateSystem("interpolatedPoseGraph", yoInterpolatedPose, 1.0, YoAppearance.Blue());
      yoGraphicsListRegistry.registerYoGraphic("outputs", interpolatedPoseGraphic);

      alphaFilterBreakFrequency.set(0.6);
      ClippedSpeedOffsetErrorInterpolator interpolator = new ClippedSpeedOffsetErrorInterpolator(registry, referenceFrameToBeCorrected, 0.001);
      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(0, timeStampedTransform );
      referenceFrameToBeCorrectedTransform.set(timeStampedTransform.getTransform3D());
      referenceFrameToBeCorrected.update();
      interpolator.setInterpolatorInputs(startPose, goalPose, 1.0);
      
      for(long timestamp = 0; timestamp <maxTimeStamp; timestamp++)
      {
         referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.findTransform(timestamp, timeStampedTransform );
         referenceFrameToBeCorrectedTransform.set(timeStampedTransform.getTransform3D());
         referenceFrameToBeCorrected.update();
         
         referenceFrameToBeCorrectedPose.setToZero(referenceFrameToBeCorrected);
         referenceFrameToBeCorrectedPose.changeFrame(worldFrame);
         yoReferenceFrameToBeCorrectedPose.set(referenceFrameToBeCorrectedPose);
         
         interpolator.interpolateError(interpolatedPose);
         yoInterpolatedPose.set(interpolatedPose);

         ////////////////update startPose and goalPose////
         referenceFrameToBeCorrectedTransform.getTranslation(referenceFrameToBeCorrectedTransform_Translation);
         referenceFrameToBeCorrectedTransform.getRotation(referenceFrameToBeCorrectedTransform_Rotation);
         
         startPoseTransform.setIdentity();
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), referenceFrameToBeCorrectedTransform_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), startPose_Translation));
         startPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         startPoseTransform.multiply(new RigidBodyTransform(startPose_Rotation, new Vector3D()));
         
         startPose.set(startPoseTransform);
         yoStartPose.set(startPose);
         
         
         goalPoseTransform.setIdentity();
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), referenceFrameToBeCorrectedTransform_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(new Quaternion(), goalPose_Translation));
         goalPoseTransform.multiply(new RigidBodyTransform(referenceFrameToBeCorrectedTransform_Rotation, new Vector3D()));
         goalPoseTransform.multiply(new RigidBodyTransform(goalPose_Rotation, new Vector3D()));
         goalPose.set(goalPoseTransform);
         yoGoalPose.set(goalPose);
         /////////////////////////////////////////////////
         
         this.timeStamp.set(timestamp);
         scs.tickAndUpdate();
      }
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);   
      
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private void generateKnownTranslationOnlyReferenceFrameToBeCorrectedWaypoints(Vector3D startLocationInWorld, Vector3D endLocationInWorld)
   {
      long timeStamp;
      Quaternion rotation = new Quaternion();

      timeStamp = 0;
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, startLocationInWorld, rotation);
      
      timeStamp = maxTimeStamp;
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, endLocationInWorld, rotation);
   }
   
   private void generateKnownRotationOnlyReferenceFrameToBeCorrectedWaypoints(Quaternion startOrientationInWorld, Quaternion endOrientationInWorld)
   {
      long timeStamp;
      Vector3D translation = new Vector3D();
      
      timeStamp = 0;
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, translation, startOrientationInWorld);
      
      timeStamp = maxTimeStamp;
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, translation, endOrientationInWorld);
   }
   
   private void generateKnownReferenceFrameToBeCorrectedWaypoints(Vector3D startLocationInWorld, Vector3D endLocationInWorld, Quaternion startOrientationInWorld, Quaternion endOrientationInWorld)
   {
      long timeStamp;
      
      timeStamp = 0;
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, startLocationInWorld, startOrientationInWorld);
      
      timeStamp = maxTimeStamp;
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, endLocationInWorld, endOrientationInWorld);
   }
   
   private void generateRandomReferenceFrameToBeCorrectedWaypoints()
   {
      Random random = new Random();
      long timeStamp;
      Vector3D translation = new Vector3D();
      Quaternion rotation = new Quaternion();
      
      timeStamp = 0;
      translation = RandomGeometry.nextVector3D(random);
      rotation = RandomGeometry.nextQuaternion(random);
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, translation, rotation);
      
      timeStamp = maxTimeStamp;
      translation = RandomGeometry.nextVector3D(random);
      rotation = RandomGeometry.nextQuaternion(random);
      putReferenceFrameToBeCorrectedWaypointInTransformBuffer(timeStamp, translation, rotation);
   }

   private void putReferenceFrameToBeCorrectedWaypointInTransformBuffer(long timeStamp, Vector3D translation, Quaternion rotation)
   {
      RigidBodyTransform temporaryReferenceFrameToBeCorrectedTransformInWorldFrame = new RigidBodyTransform();  
      temporaryReferenceFrameToBeCorrectedTransformInWorldFrame.setTranslation(translation);
      temporaryReferenceFrameToBeCorrectedTransformInWorldFrame.setRotation(rotation);
      referenceFrameToBeCorrectedWaypointsTransformPoseBufferInWorldFrame.put(temporaryReferenceFrameToBeCorrectedTransformInWorldFrame, timeStamp);
   }
   
   public static void main(String[] args)
   {
      new ClippedSpeedOffsetErrorInterpolatorVisualizer();
   }
}
