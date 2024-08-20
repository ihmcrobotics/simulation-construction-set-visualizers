package us.ihmc.scsVisualizers.kinematicsBasedStateEstimation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.OutdatedPoseToUpToDateReferenceFrameUpdater;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class OutdatedPoseToUpToDateReferenceFrameUpdaterVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final YoLong timeStampYoVariable = new YoLong("timeStamp", registry);

   public OutdatedPoseToUpToDateReferenceFrameUpdaterVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      int numberOfUpToDateTransforms = 10;
      int numberOfOutdatedTransforms = 3;
      long firstTimeStamp = 1000;
      long lastTimeStamp = 2000;
      long[] oudatedTimeStamps = new long[numberOfOutdatedTransforms];

      Random random = new Random(1987L);

      FramePose3D upToDatePoseInPresent = new FramePose3D(worldFrame);
      PoseReferenceFrame upToDateReferenceFrameInPresent = new PoseReferenceFrame("upToDateReferenceFrameInPresent", upToDatePoseInPresent);
      OutdatedPoseToUpToDateReferenceFrameUpdater outdatedPoseToUpToDateReferenceFrameUpdater = new OutdatedPoseToUpToDateReferenceFrameUpdater(
            numberOfUpToDateTransforms, upToDateReferenceFrameInPresent);
      ReferenceFrame outdatedReferenceFrame_InUpToDateReferenceFrame;
      outdatedReferenceFrame_InUpToDateReferenceFrame = outdatedPoseToUpToDateReferenceFrameUpdater.getLocalizationReferenceFrameToBeUpdated();
      
      TimeStampedTransformBuffer upToDateTimeStampedTransformPoseBuffer = new TimeStampedTransformBuffer(numberOfUpToDateTransforms);
      TimeStampedTransformBuffer outdatedTimeStampedTransformBuffer = new TimeStampedTransformBuffer(numberOfOutdatedTransforms);

      YoFramePoseUsingYawPitchRoll upToDateYoPoseInPresent = new YoFramePoseUsingYawPitchRoll("upToDateYoPoseInPresent", worldFrame, registry);
      YoGraphicCoordinateSystem upToDateYoPoseInPresentGraphic = new YoGraphicCoordinateSystem("upToDateYoPoseInPresentGraph", upToDateYoPoseInPresent, 0.5,
            YoAppearance.Yellow());
      yoGraphicsListRegistry.registerYoGraphic("upToDatePoses", upToDateYoPoseInPresentGraphic);

      YoFramePoseUsingYawPitchRoll lastOutdatedPoseInPresentYoPose = new YoFramePoseUsingYawPitchRoll("lastOutdatedPoseInPresentYoPose", worldFrame, registry);
      YoGraphicCoordinateSystem lastOutdatedPoseInPresentYoPoseGraphic = new YoGraphicCoordinateSystem("lastOutdatedPoseInPresentYoPoseGraph",
            lastOutdatedPoseInPresentYoPose, 0.5, YoAppearance.Purple());
      yoGraphicsListRegistry.registerYoGraphic("outdatedPosesInPresentAtTimeStamp", lastOutdatedPoseInPresentYoPoseGraphic);

      YoFramePoseUsingYawPitchRoll outdatedPoseExpressedInUpToDateReferenceFrameYoPose = new YoFramePoseUsingYawPitchRoll("outdatedPoseExpressedInUpToDateReferenceFrameYoPose", worldFrame,
            registry);
      YoGraphicCoordinateSystem outdatedPoseExpressedInUpToDateReferenceFrameYoPoseGraphic = new YoGraphicCoordinateSystem(
            "outdatedPoseExpressedInUpToDateReferenceFrameYoPoseGraph", outdatedPoseExpressedInUpToDateReferenceFrameYoPose, 0.5, YoAppearance.White());
      yoGraphicsListRegistry.registerYoGraphic("outdatedPosesInPresent", outdatedPoseExpressedInUpToDateReferenceFrameYoPoseGraphic);

      for (int i = 0; i < numberOfUpToDateTransforms; i++)
      {
         RigidBodyTransform currentUpToDateTransform = new RigidBodyTransform();
         long currentTimeStamp = (long) (i * (lastTimeStamp - firstTimeStamp) / numberOfUpToDateTransforms + firstTimeStamp);
         currentUpToDateTransform.getTranslation().set((double) (20.0 / numberOfUpToDateTransforms * i) - 5.0, RandomNumbers.nextDouble(random, -1.5, 1.5), RandomNumbers.nextDouble(random, 0.1, 2.0));
         currentUpToDateTransform.getRotation().set(EuclidCoreRandomTools.nextQuaternion(random));
         upToDateTimeStampedTransformPoseBuffer.put(currentUpToDateTransform, currentTimeStamp);
         outdatedPoseToUpToDateReferenceFrameUpdater.putStateEstimatorTransformInBuffer(currentUpToDateTransform, currentTimeStamp);

         YoFramePoseUsingYawPitchRoll currentUpToDateTransformYoFramePose = new YoFramePoseUsingYawPitchRoll("currentUpToDateTransformYoFramePose_" + i, worldFrame, registry);
         FramePose3D temporaryFramePose = new FramePose3D(worldFrame, currentUpToDateTransform);
         currentUpToDateTransformYoFramePose.set(temporaryFramePose);
         YoGraphicCoordinateSystem upToDateTransformGraphic = new YoGraphicCoordinateSystem("upToDateTransformGraph_" + i,
               currentUpToDateTransformYoFramePose, 0.6, YoAppearance.Blue());
         yoGraphicsListRegistry.registerYoGraphic("UpToDateTransforms", upToDateTransformGraphic);
      }

      for (int j = 0; j < numberOfOutdatedTransforms; j++)
      {
         TimeStampedTransform3D upToDateTimeStampedTransformInPast = new TimeStampedTransform3D();
         AxisAngle errorRotationOffset = EuclidCoreRandomTools.nextAxisAngle(random, 0.6);
         Vector3D errorTranslationOffset = new Vector3D(0.05 * j, 0.0, 0.2 * j);
//         Quat4d errorRotationOffset = new Quat4d();
//         RotationFunctions.setQuaternionBasedOnYawPitchRoll(errorRotationOffset,Math.toRadians(95.0), 0.0, 0.0);
         RigidBodyTransform offsetRotationTransform = new RigidBodyTransform(errorRotationOffset, new Vector3D());

//         Vector3d errorTranslationOffset = new Vector3d(0.0,0.0,0.0);
         RigidBodyTransform offsetTranslationTransform = new RigidBodyTransform(new Quaternion(), errorTranslationOffset);
         
         RigidBodyTransform finalErrorTransform = new RigidBodyTransform();
         
         long timeStamp = (long) (j * (lastTimeStamp * 0.8 - firstTimeStamp * 0.8) / numberOfOutdatedTransforms + firstTimeStamp * 1.2);
         upToDateTimeStampedTransformPoseBuffer.findTransform(timeStamp, upToDateTimeStampedTransformInPast);

         oudatedTimeStamps[j] = timeStamp;

         RigidBodyTransform upToDateTransformInPast_Translation = new RigidBodyTransform(upToDateTimeStampedTransformInPast.getTransform3D());
         RigidBodyTransform upToDateTransformInPast_Rotation = new RigidBodyTransform(upToDateTransformInPast_Translation);
         upToDateTransformInPast_Translation.getRotation().setToZero();
         upToDateTransformInPast_Rotation.getTranslation().setToZero();
         
         finalErrorTransform.multiply(upToDateTransformInPast_Translation);
         finalErrorTransform.multiply(offsetTranslationTransform);
         finalErrorTransform.multiply(upToDateTransformInPast_Rotation);
         finalErrorTransform.multiply(offsetRotationTransform);
         
         outdatedTimeStampedTransformBuffer.put(finalErrorTransform, timeStamp);

         YoFramePoseUsingYawPitchRoll currentOutdatedTransformYoFramePose = new YoFramePoseUsingYawPitchRoll("currentOutdatedTransformYoFramePose_" + j, worldFrame, registry);
         FramePose3D temporaryFramePose = new FramePose3D(worldFrame, finalErrorTransform);
         currentOutdatedTransformYoFramePose.set(temporaryFramePose);
         YoGraphicCoordinateSystem currentOutdatedTransformYoFramePoseGraphic = new YoGraphicCoordinateSystem("currentOutdatedTransformYoFramePoseGraphic_" + j, currentOutdatedTransformYoFramePose, 0.3,
               YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("outdatedTransforms", currentOutdatedTransformYoFramePoseGraphic);

      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(linkGraphics);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      TimeStampedTransform3D timeStampTransform3DMoving = new TimeStampedTransform3D();

      int outdatedTimeStampsIndex = 0;

      FramePose3D outdatedPoseExpressedInUpToDateReferenceFrame = new FramePose3D(worldFrame);

      for (long timeStamp = firstTimeStamp; timeStamp < lastTimeStamp; timeStamp++)
      {
         timeStampYoVariable.set(timeStamp);
         upToDateTimeStampedTransformPoseBuffer.findTransform(timeStamp, timeStampTransform3DMoving);
         upToDatePoseInPresent.set(timeStampTransform3DMoving.getTransform3D());
         upToDateReferenceFrameInPresent.setPoseAndUpdate(upToDatePoseInPresent);

         if (outdatedTimeStampsIndex < numberOfOutdatedTransforms && oudatedTimeStamps[outdatedTimeStampsIndex] == timeStamp - 100)
         {
            TimeStampedTransform3D outdatedTimeStampedTransform = new TimeStampedTransform3D();
            outdatedTimeStampedTransformBuffer.findTransform(oudatedTimeStamps[outdatedTimeStampsIndex], outdatedTimeStampedTransform);
            outdatedPoseToUpToDateReferenceFrameUpdater.updateLocalizationTransform(outdatedTimeStampedTransform);
            FramePose3D lastUpdateOfOutdatedPoseInPresent = new FramePose3D(outdatedReferenceFrame_InUpToDateReferenceFrame);
            lastUpdateOfOutdatedPoseInPresent.changeFrame(worldFrame);
            lastOutdatedPoseInPresentYoPose.set(lastUpdateOfOutdatedPoseInPresent);
            outdatedTimeStampsIndex++;
         }

         outdatedReferenceFrame_InUpToDateReferenceFrame.update();
         outdatedPoseExpressedInUpToDateReferenceFrame.setToZero(outdatedReferenceFrame_InUpToDateReferenceFrame);
         outdatedPoseExpressedInUpToDateReferenceFrame.changeFrame(worldFrame);
         
         outdatedPoseExpressedInUpToDateReferenceFrameYoPose.set(outdatedPoseExpressedInUpToDateReferenceFrame);

         upToDateYoPoseInPresent.set(upToDatePoseInPresent);
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new OutdatedPoseToUpToDateReferenceFrameUpdaterVisualizer();
   }

}
