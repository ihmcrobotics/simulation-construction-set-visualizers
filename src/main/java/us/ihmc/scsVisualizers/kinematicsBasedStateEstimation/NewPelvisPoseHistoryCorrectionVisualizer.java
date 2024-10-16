package us.ihmc.scsVisualizers.kinematicsBasedStateEstimation;

import java.util.Random;

import controller_msgs.msg.dds.LocalizationPacket;
import controller_msgs.msg.dds.PelvisPoseErrorPacket;
import ihmc_common_msgs.msg.dds.StampedPosePacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.TimeStampedTransformBuffer;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ClippedSpeedOffsetErrorInterpolatorParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.NewPelvisPoseHistoryCorrection;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoLong;

public class NewPelvisPoseHistoryCorrectionVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoRegistry registry;

   private Robot robot;
   private ReferenceFrame pelvisReferenceFrame;
   private RigidBodyTransform robotTransformInWorldFrame = new RigidBodyTransform();
   private SixDoFJoint sixDofPelvisJoint;

   private final int pelvisBufferSize = 100;
   private final double estimatorDT = 0.001;
   private final double simulationDuration = 60.0;
   private final int numberOfTimeStamps = (int) (simulationDuration / estimatorDT) + 1;

   private SimulationConstructionSet simulationConstructionSet;
   private final int bufferSize = numberOfTimeStamps;

   private ExternalPelvisPoseCreator externalPelvisPoseCreator;

   private final FramePose3D pelvisPose = new FramePose3D(worldFrame);
   private final YoFramePoseUsingYawPitchRoll yoPelvisPose;
   int numberOfPelvisWaypoints = 11;
   TimeStampedTransformBuffer pelvisWaypointsTransformPoseBufferInWorldFrame = new TimeStampedTransformBuffer(numberOfPelvisWaypoints);

   int numberOfBigIcpOffsets = 10;
   TimeStampedTransformBuffer icpBigOffsetTransformPoseBufferInPelvisFrame = new TimeStampedTransformBuffer(numberOfBigIcpOffsets);
   TimeStampedTransformBuffer allIcpTransformPoseBufferInWorldFrame = new TimeStampedTransformBuffer(70);
   private final FramePose3D icpPose = new FramePose3D(worldFrame);
   private final YoFramePoseUsingYawPitchRoll yoIcpPose;

   private NewPelvisPoseHistoryCorrection pelvisCorrector;
   private final FramePose3D correctedPelvisPoseInWorldFrame = new FramePose3D(worldFrame);
   private final RigidBodyTransform correctedPelvisTransformInWorldFrame = new RigidBodyTransform();
   private final YoFramePoseUsingYawPitchRoll yoCorrectedPelvisPose;
   
   public NewPelvisPoseHistoryCorrectionVisualizer() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      setupRobot();
      registry = robot.getRobotsYoRegistry();
      setupSim();
      setupCorrector(yoGraphicsListRegistry);

      generatePelvisWayPoints();
      generateBigIcpOffsets();
      generateAllIcpTransformsWithTheBigIcpOffsetsAsBasis();

      yoPelvisPose = new YoFramePoseUsingYawPitchRoll("vizPelvisPose", worldFrame, registry);
      YoGraphicCoordinateSystem pelvisPoseGraphic = new YoGraphicCoordinateSystem("pelvisPoseGraphic", yoPelvisPose, 0.5, YoAppearance.Grey());
      yoGraphicsListRegistry.registerYoGraphic("pelvisPose", pelvisPoseGraphic);

      yoIcpPose = new YoFramePoseUsingYawPitchRoll("vizIcpPose", worldFrame, registry);
      YoGraphicCoordinateSystem icpPoseGraphic = new YoGraphicCoordinateSystem("icpPoseGraphic", yoIcpPose, 0.5, YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("icpPose", icpPoseGraphic);

      yoCorrectedPelvisPose = new YoFramePoseUsingYawPitchRoll("vizCorrectedPelvisPose", worldFrame , registry);
      YoGraphicCoordinateSystem correctedPelvisPoseGraphic = new YoGraphicCoordinateSystem("correctedPelvisPoseGraphic", yoCorrectedPelvisPose, 0.8,YoAppearance.Yellow());
      yoGraphicsListRegistry.registerYoGraphic("correctedPelvis", correctedPelvisPoseGraphic);
      
      YoLong yoTimeStamp = new YoLong("timeStamp", registry);
      TimeStampedTransform3D pelvisTimeStampedTransform3D = new TimeStampedTransform3D();
      TimeStampedTransform3D icpTimeStampedTransform3D = new TimeStampedTransform3D();
      for (long timeStamp = 0; timeStamp < numberOfTimeStamps; timeStamp++)
      {
         yoTimeStamp.set(timeStamp);
         pelvisWaypointsTransformPoseBufferInWorldFrame.findTransform(timeStamp, pelvisTimeStampedTransform3D);
         robotTransformInWorldFrame.set(pelvisTimeStampedTransform3D.getTransform3D());
         pelvisReferenceFrame.update();

         sixDofPelvisJoint.setJointConfiguration(pelvisTimeStampedTransform3D.getTransform3D());
         sixDofPelvisJoint.updateFramesRecursively();
         pelvisPose.set(sixDofPelvisJoint.getJointPose());
         yoPelvisPose.set(pelvisPose);

         pelvisCorrector.doControl(timeStamp);

         sixDofPelvisJoint.getJointConfiguration(correctedPelvisTransformInWorldFrame);
         correctedPelvisPoseInWorldFrame.set(correctedPelvisTransformInWorldFrame);
         yoCorrectedPelvisPose.set(correctedPelvisPoseInWorldFrame);

         if ( timeStamp > 3000 && ((timeStamp - 80) % 1000) == 0)
         {
            allIcpTransformPoseBufferInWorldFrame.findTransform(timeStamp - 80, icpTimeStampedTransform3D);
            icpPose.set(icpTimeStampedTransform3D.getTransform3D());

            yoIcpPose.set(icpPose);

            StampedPosePacket newestStampedPosePacket = HumanoidMessageTools.createStampedPosePacket("/playback", icpTimeStampedTransform3D, 1.0);

            externalPelvisPoseCreator.setNewestPose(newestStampedPosePacket);

         }

         
         simulationConstructionSet.tickAndUpdate();
      }

      simulationConstructionSet.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      simulationConstructionSet.startOnAThread();
      ThreadTools.sleepForever();
   }

   private void generatePelvisWayPoints()
   {
      RigidBodyTransform pelvisTransformInWorldFrame = new RigidBodyTransform();
      long timeStamp;
      Vector3D translation = new Vector3D();
      Quaternion rotation = new Quaternion();

      timeStamp = 0;
      translation.set(0.0, 0.0, 0.7);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 5000;
      translation.set(1.0, 0.0, 0.5);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 10000;
      translation.set(1.0, 1.0, 0.7);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(5.0), Math.toRadians(5.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 15000;
      translation.set(1.0, 1.0, 0.6);
      rotation.setYawPitchRoll(Math.toRadians(-45.0), Math.toRadians(0.0), Math.toRadians(-1.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 20000;
      translation.set(3.0, -1.0, 0.6);
      rotation.setYawPitchRoll(Math.toRadians(-45.0), Math.toRadians(-12.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 25000;
      translation.set(3.0, -1.0, 0.6);
      rotation.setYawPitchRoll(Math.toRadians(135.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 30000;
      translation.set(1.0, 1.0, 0.9);
      rotation.setYawPitchRoll(Math.toRadians(135.0), Math.toRadians(0.0), Math.toRadians(-2.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 35000;
      translation.set(1.0, 1.0, 0.9);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-15.0), Math.toRadians(5.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 40000;
      translation.set(-1.0, 1.0, 0.5);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(-1.0), Math.toRadians(-5.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 45000;
      translation.set(-1.0, -1.0, 0.7);
      rotation.setYawPitchRoll(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);

      timeStamp = 50000;
      translation.set(-1.0, -1.0, 0.2);
      rotation.setYawPitchRoll(Math.toRadians(180.0), Math.toRadians(10.0), Math.toRadians(10.0));
      putPelvisWaypointInTransformBuffer(pelvisTransformInWorldFrame, timeStamp, translation, rotation);
   }

   private void putPelvisWaypointInTransformBuffer(RigidBodyTransform pelvisTransformInWorldFrame, long timeStamp, Vector3D translation, Quaternion rotation)
   {
      pelvisTransformInWorldFrame.getTranslation().set(translation);
      pelvisTransformInWorldFrame.getRotation().set(rotation);
      pelvisWaypointsTransformPoseBufferInWorldFrame.put(pelvisTransformInWorldFrame, timeStamp);
   }

   private void generateBigIcpOffsets()
   {
      long timeStamp;

      Vector3D translationOffset = new Vector3D();
      Quaternion rotationOffset = new Quaternion();

      timeStamp = 3000;
      translationOffset.set(0.1, -0.05, 0.08);
      rotationOffset.setYawPitchRoll(0.0, 0.0, 0.0);
      putBigIcpOffsetInTransformBuffer(timeStamp, translationOffset, rotationOffset);

      timeStamp = 9000;
      translationOffset.set(0.1, -0.05, 0.08);
      rotationOffset.setYawPitchRoll(0.05, -0.08, 0.15);
      putBigIcpOffsetInTransformBuffer(timeStamp, translationOffset, rotationOffset);

      timeStamp = 25000;
      translationOffset.set(0.20, -0.15, -0.02);
      rotationOffset.setYawPitchRoll(-0.05, 0.0, 0.10);
      putBigIcpOffsetInTransformBuffer(timeStamp, translationOffset, rotationOffset);

      timeStamp = 35000;
      translationOffset.set(-0.10, -0.07,0.02);
      rotationOffset.setYawPitchRoll(-0.08, 0.05, -0.01);
      putBigIcpOffsetInTransformBuffer(timeStamp, translationOffset, rotationOffset);

      timeStamp = 38000;
      translationOffset.set(-0.18, 0.05, -0.04);
      rotationOffset.setYawPitchRoll(0.0, 0.05, -0.05);
      putBigIcpOffsetInTransformBuffer(timeStamp, translationOffset, rotationOffset);

   }

   private void putBigIcpOffsetInTransformBuffer(long timeStamp, Vector3D translationOffset, Quaternion rotationOffset)
   {
      RigidBodyTransform icpTransformInPelvisFrame = new RigidBodyTransform();
      icpTransformInPelvisFrame.set(rotationOffset, translationOffset);
      icpBigOffsetTransformPoseBufferInPelvisFrame.put(icpTransformInPelvisFrame, timeStamp);
   }

   private void generateAllIcpTransformsWithTheBigIcpOffsetsAsBasis()
   {
      Random random = new Random(4242L);
      putIcpTranformInBufferWithBigOffsetAtFirstTimeStamp(3000, 9000, random);
      putIcpTranformInBufferWithBigOffsetAtFirstTimeStamp(9000, 25000, random);
      putIcpTranformInBufferWithBigOffsetAtFirstTimeStamp(25000, 35000, random);
      putIcpTranformInBufferWithBigOffsetAtFirstTimeStamp(35000, 38000, random);
      putIcpTranformInBufferWithBigOffsetAtFirstTimeStamp(38000, 51000, random);
   }

   private void putIcpTranformInBufferWithBigOffsetAtFirstTimeStamp(long firstTimeStamp, long lastTimeStamp, Random random)
   {
      TimeStampedTransform3D pelvisTransformAtSpecificTimeStamp = new TimeStampedTransform3D();
      RigidBodyTransform pelvisTransformAtSpecificTimeStamp_Translation = new RigidBodyTransform();
      RigidBodyTransform pelvisTransformAtSpecificTimeStamp_Rotation = new RigidBodyTransform();
      
      TimeStampedTransform3D currentIcpTimeStampedTransform = new TimeStampedTransform3D();
      RigidBodyTransform currentIcpWithBigOffsetTransform_Translation = new RigidBodyTransform();
      RigidBodyTransform currentIcpWithBigOffsetTransform_Rotation = new RigidBodyTransform();
      
      RigidBodyTransform currentIcpModifiedWithSmallOffsetsTransform = new RigidBodyTransform();
      RigidBodyTransform smallOffsetsTransform_Translation = new RigidBodyTransform();
      RigidBodyTransform smallOffsetsTransform_Rotation = new RigidBodyTransform();

      //set the big offset
      pelvisWaypointsTransformPoseBufferInWorldFrame.findTransform(firstTimeStamp, pelvisTransformAtSpecificTimeStamp);
      
      pelvisTransformAtSpecificTimeStamp_Translation.set(pelvisTransformAtSpecificTimeStamp.getTransform3D());
      pelvisTransformAtSpecificTimeStamp_Rotation.set(pelvisTransformAtSpecificTimeStamp_Translation);
      pelvisTransformAtSpecificTimeStamp_Translation.getRotation().setToZero();
      pelvisTransformAtSpecificTimeStamp_Rotation.getTranslation().setToZero();

      icpBigOffsetTransformPoseBufferInPelvisFrame.findTransform(firstTimeStamp, currentIcpTimeStampedTransform);
      currentIcpWithBigOffsetTransform_Translation.set(currentIcpTimeStampedTransform.getTransform3D());
      currentIcpWithBigOffsetTransform_Rotation.set(currentIcpWithBigOffsetTransform_Translation);
      currentIcpWithBigOffsetTransform_Translation.getRotation().setToZero();
      currentIcpWithBigOffsetTransform_Rotation.getTranslation().setToZero();

      RigidBodyTransform firstTimeStamBigIcpOffsetTransform = new RigidBodyTransform();
      firstTimeStamBigIcpOffsetTransform.multiply(pelvisTransformAtSpecificTimeStamp_Translation);
      firstTimeStamBigIcpOffsetTransform.multiply(currentIcpWithBigOffsetTransform_Translation);
      firstTimeStamBigIcpOffsetTransform.multiply(pelvisTransformAtSpecificTimeStamp_Rotation);
      firstTimeStamBigIcpOffsetTransform.multiply(currentIcpWithBigOffsetTransform_Rotation);
      
      allIcpTransformPoseBufferInWorldFrame.put(firstTimeStamBigIcpOffsetTransform, firstTimeStamp);

      for (long timeStamp = firstTimeStamp + 1000; timeStamp < lastTimeStamp; timeStamp += 1000)
      {
         Vector3D smallTranslationOffset = EuclidCoreRandomTools.nextVector3D(random, 0.02);
         Quaternion smallRotationOffset = EuclidCoreRandomTools.nextQuaternion(random, 0.02);
         
         pelvisWaypointsTransformPoseBufferInWorldFrame.findTransform(timeStamp, pelvisTransformAtSpecificTimeStamp);
         pelvisTransformAtSpecificTimeStamp_Translation.set(pelvisTransformAtSpecificTimeStamp.getTransform3D());
         pelvisTransformAtSpecificTimeStamp_Rotation.set(pelvisTransformAtSpecificTimeStamp_Translation);
         pelvisTransformAtSpecificTimeStamp_Translation.getRotation().setToZero();
         pelvisTransformAtSpecificTimeStamp_Rotation.getTranslation().setToZero();
         
         Vector3D currentIcpBigTranslationOffset = new Vector3D();
         Quaternion currentIcpBigRotationOffset = new Quaternion();
         currentIcpBigTranslationOffset.set(currentIcpWithBigOffsetTransform_Translation.getTranslation());
         currentIcpBigRotationOffset.set(currentIcpWithBigOffsetTransform_Rotation.getRotation());
         smallTranslationOffset.add(currentIcpBigTranslationOffset);
         smallRotationOffset.preMultiply(currentIcpBigRotationOffset);

         smallOffsetsTransform_Translation.setTranslationAndIdentityRotation(smallTranslationOffset);
         smallOffsetsTransform_Rotation.setRotationAndZeroTranslation(smallRotationOffset);
         
         currentIcpModifiedWithSmallOffsetsTransform.setIdentity();
         currentIcpModifiedWithSmallOffsetsTransform.multiply(pelvisTransformAtSpecificTimeStamp_Translation);
         currentIcpModifiedWithSmallOffsetsTransform.multiply(smallOffsetsTransform_Translation);
         currentIcpModifiedWithSmallOffsetsTransform.multiply(pelvisTransformAtSpecificTimeStamp_Rotation);
         currentIcpModifiedWithSmallOffsetsTransform.multiply(smallOffsetsTransform_Rotation);
         
         allIcpTransformPoseBufferInWorldFrame.put(currentIcpModifiedWithSmallOffsetsTransform, timeStamp);
      }
   }

   private void setupRobot()
   {
      robot = new Robot("dummy");

      pelvisReferenceFrame = new ReferenceFrame("playback", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(robotTransformInWorldFrame);
         }
      };
      RigidBodyBasics rigidBody = new RigidBody("playback", pelvisReferenceFrame);
      sixDofPelvisJoint = new SixDoFJoint("playback", rigidBody);
   }

   private void setupSim()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);

      simulationConstructionSet = new SimulationConstructionSet(robot, parameters);
      simulationConstructionSet.setDT(0.001, 1);
   }

   private void setupCorrector(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      externalPelvisPoseCreator = new ExternalPelvisPoseCreator();
      pelvisCorrector = new NewPelvisPoseHistoryCorrection(sixDofPelvisJoint, estimatorDT, registry, pelvisBufferSize, yoGraphicsListRegistry, externalPelvisPoseCreator, new ClippedSpeedOffsetErrorInterpolatorParameters());
   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      new NewPelvisPoseHistoryCorrectionVisualizer();
   }

   /////////////////////////////////////////////////////////
   /////// Necessary class to make the visualizer work //////
   /////////////////////////////////////////////////////////
   private class ExternalPelvisPoseCreator implements PelvisPoseCorrectionCommunicatorInterface
   {
      private StampedPosePacket newestStampedPosePacket;
      boolean newPose;

      public void setNewestPose(StampedPosePacket newestStampedPosePacket)
      {
         this.newestStampedPosePacket = newestStampedPosePacket;
         newPose = true;
      }

      @Override
      public boolean hasNewPose()
      {
         return newPose;
      }

      @Override
      public StampedPosePacket getNewExternalPose()
      {
         newPose = false;
         return this.newestStampedPosePacket;
      }

      @Override
      public void receivedPacket(StampedPosePacket object)
      {
         //doNothing
      }

      @Override
      public void sendPelvisPoseErrorPacket(PelvisPoseErrorPacket pelvisPoseErrorPacket)
      {
         //doNothing
      }

      @Override
      public void sendLocalizationResetRequest(LocalizationPacket localizationPacket)
      {
         System.out.println("orientationError too big");
         System.out.println("reseting the localization map");
         
      }
   }
}
