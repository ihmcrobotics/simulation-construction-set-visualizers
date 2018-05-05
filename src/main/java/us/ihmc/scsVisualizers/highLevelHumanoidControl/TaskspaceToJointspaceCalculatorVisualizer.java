package us.ihmc.scsVisualizers.highLevelHumanoidControl;

import java.util.LinkedHashMap;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.CirclePoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoInteger;

public class TaskspaceToJointspaceCalculatorVisualizer
{
   private static final int BUFFER_SIZE = 10000;
   private static final boolean BENCHMARK = false;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFramePoseUsingYawPitchRoll finalHandPose = new YoFramePoseUsingYawPitchRoll("finalHandPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll initialHandPose = new YoFramePoseUsingYawPitchRoll("initialHandPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll currentHandPose = new YoFramePoseUsingYawPitchRoll("currentHandPose", worldFrame, registry);

   private static final boolean WIGGLE_PELVIS_XYZ = true;
   private final double pelvisXYZNoise = 0.0005;
   private final double amplitudeWigglePelvisXYZ = 0.0;
   private final double frequencyWigglePelvisXYZ = 0.1;
   private static final boolean WIGGLE_PELVIS_YAW_PITCH_ROLL = true;
   private final double pelvisYPRNoise = 0.0005;
   private final double amplitudeWigglePelvisYPR = 0.005;
   private final double frequencyWigglePelvisYPR = 16.0;
   private static final boolean NOISE_ON_BACK_JOINTS = true;
   private final double noiseAmplitudeForBackJoints = 0.005;

   private final YoDouble[] errorSpatial;
   private final YoDouble errorScalar = new YoDouble("errorScalar", registry);
   private final YoInteger numberOfSuccess = new YoInteger("numberOfSuccess", registry);
   private final Random random = new Random(44656L);

   private static final int numberOfRuns = 20;
   private static final int numberOfIterations = BUFFER_SIZE / numberOfRuns - 1;
   private static final double dt = 0.004;
   private static final double trajectoryTime = 0.5 * numberOfIterations * dt;

   private StraightLinePoseTrajectoryGenerator straightLineTrajectory;
   private CirclePoseTrajectoryGenerator circleTrajectory;
   private SimulationConstructionSet scs;
   private OneDoFJoint[] armJoints;

   private final FramePose3D handPose = new FramePose3D();

   private final int numberOfConstraints = 6;

   private final DenseMatrix64F selectionMatrix = new DenseMatrix64F(numberOfConstraints, 6);
   private FullHumanoidRobotModel fullRobotModel;
   private JointAnglesWriter jointAnglesWriter;
   private ReferenceFrame handControlFrame;
   private AtlasRobotModel robotModel;

   private TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator;
   private final DenseMatrix64F jointAngles = new DenseMatrix64F(7, 1);

   private final YoDouble yoTime;
   private final FrameVector3D desiredHandLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredHandAngularVelocity = new FrameVector3D();
   private final Twist desiredHandTwist = new Twist();

   private final LinkedHashMap<OneDoFJoint, FilteredVelocityYoVariable> qd_fds = new LinkedHashMap<>();
   private final LinkedHashMap<OneDoFJoint, FilteredVelocityYoVariable> qdd_fds = new LinkedHashMap<>();

   private int counter = 0;
   private final long startTime, endTime;

   private final FramePose3D initialPelvisPose = new FramePose3D();
   private final FramePose3D pelvisPoseWithWiggle = new FramePose3D();

   public TaskspaceToJointspaceCalculatorVisualizer()
   {
      CommonOps.setIdentity(selectionMatrix);
//      selectionMatrix.set(0, 0, 0.0);
//      selectionMatrix.set(0, 1, 1.0);
//      selectionMatrix.set(1, 2, 1.0);
//      selectionMatrix.set(0, 3, 1.0);
//      selectionMatrix.set(1, 4, 1.0);
//      selectionMatrix.set(2, 5, 1.0);

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      fullRobotModel = robotModel.createFullRobotModel();
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, robotModel.getJointMap());
      yoTime = robot.getYoTime();

      SDFPerfectSimulatedSensorReader sensorReader = new SDFPerfectSimulatedSensorReader(robot, fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));
      sensorReader.read();

      fullRobotModel.updateFrames();
      initialPelvisPose.setToZero(fullRobotModel.getRootJoint().getFrameAfterJoint());
      initialPelvisPose.changeFrame(worldFrame);
      jointAnglesWriter = new JointAnglesWriter(robot, fullRobotModel);

      RigidBody chest = fullRobotModel.getChest();
      RigidBody leftHand = fullRobotModel.getHand(RobotSide.LEFT);
      ReferenceFrame leftHandFrame = leftHand.getBodyFixedFrame();
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();

      YoGraphicsListRegistry yoGraphicsListRegistry;
      if (!BENCHMARK)
      {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      scs = new SimulationConstructionSet(robot, parameters);
      scs.setPlaybackRealTimeRate(1.0);
      scs.setDT(dt, 1);

      yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("initialPose", initialHandPose, 0.3, YoAppearance.BlueViolet()));
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("currentPose", currentHandPose, 0.3, YoAppearance.Blue()));
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("desiredPose", finalHandPose, 0.3, YoAppearance.Red()));
      }
      else
         yoGraphicsListRegistry = null;
      straightLineTrajectory = new StraightLinePoseTrajectoryGenerator("straightLineTrajectory", true, worldFrame, registry, true, yoGraphicsListRegistry);
      straightLineTrajectory.registerNewTrajectoryFrame(chestFrame);

      circleTrajectory = new CirclePoseTrajectoryGenerator("circleTrajectory", worldFrame, new ConstantDoubleProvider(trajectoryTime), registry, yoGraphicsListRegistry);
      straightLineTrajectory.setTrajectoryTime(trajectoryTime);

      if (!BENCHMARK)
      {
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.addYoVariableRegistry(registry);
      }

      taskspaceToJointspaceCalculator = new TaskspaceToJointspaceCalculator("leftHand", chest, leftHand, dt, registry);
      taskspaceToJointspaceCalculator.setupWithDefaultParameters();

      armJoints = ScrewTools.createOneDoFJointPath(chest, leftHand);

      for (OneDoFJoint joint : armJoints)
      {
         qd_fds.put(joint, new FilteredVelocityYoVariable("qd_fd_" + joint.getName(), "", 0.0, dt, registry));
         qdd_fds.put(joint, new FilteredVelocityYoVariable("qdd_fd_" + joint.getName(), "", 0.0, dt, registry));
      }

      RigidBodyTransform transformToParent = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.1, 0.0));
      handControlFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("ControlFrame", leftHandFrame, transformToParent);
      taskspaceToJointspaceCalculator.setControlFrameFixedInEndEffector(handControlFrame);
      taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrix);

      FramePoint3D circleCenter = new FramePoint3D(chestFrame, 0.6, 0.7, -0.35);
      FrameVector3D circleNormal = new FrameVector3D(chestFrame, -1.0, 0.0, 1.0);
      circleNormal.normalize();
      circleTrajectory.updateCircleFrame(circleCenter, circleNormal);
      FramePose3D initialPose = new FramePose3D(circleTrajectory.getCircleFrame());
      initialPose.setPosition(0.0, -0.175, 0.0);
      initialPose.setOrientationYawPitchRoll(0.0, Math.PI / 2.0, 0.0);
      circleTrajectory.setInitialPose(initialPose);
      circleTrajectory.setDesiredRotationAngle(-40.0 * Math.PI);
      circleTrajectory.initialize();

      errorSpatial = new YoDouble[numberOfConstraints];
      for (int i = 0; i < numberOfConstraints; i++)
      {
         errorSpatial[i] = new YoDouble("errorSpatial_" + i, registry);
      }

//      circleTrajectory.showVisualization();
      straightLineTrajectory.showVisualization();

      if (!BENCHMARK)
         scs.startOnAThread();

      startTime = System.currentTimeMillis();

      ScrewTestTools.setRandomPositionsWithinJointLimits(armJoints, random);
      handPose.setToZero(handControlFrame);
      handPose.changeFrame(chestFrame);
      handPose.setPosition(1.3 * handPose.getX(), handPose.getY(), handPose.getZ());
      handPose.changeFrame(worldFrame);
      finalHandPose.set(handPose);
      straightLineTrajectory.setFinalPose(handPose);

      for (int k = 0; k < numberOfRuns; k++)
      {
         straightLineTrajectory.switchTrajectoryFrame(worldFrame);
         finalHandPose.getFramePoseIncludingFrame(handPose);
         initialHandPose.set(handPose);
         jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
         straightLineTrajectory.setInitialPose(handPose);

         ScrewTools.getJointPositions(armJoints, jointAngles);
         ScrewTestTools.setRandomPositionsWithinJointLimits(armJoints, random);
         handPose.setToZero(handControlFrame);
         handPose.changeFrame(chestFrame);
//         handPose.setPosition(1.3 * handPose.getX(), handPose.getY(), handPose.getZ());
         handPose.changeFrame(worldFrame);
         finalHandPose.set(handPose);
         straightLineTrajectory.setFinalPose(handPose);
         ScrewTools.setJointPositions(armJoints, jointAngles);
         straightLineTrajectory.changeFrame(chestFrame);
         straightLineTrajectory.initialize();


         if (!BENCHMARK)
            scs.tickAndUpdate();

         ScrewTools.getJointPositions(armJoints, jointAngles);
         taskspaceToJointspaceCalculator.initialize(jointAngles);

         double initialTime = yoTime.getDoubleValue();

         for (int i = 0; i < numberOfIterations; i++)
         {
            applyDisturbances();

            fullRobotModel.updateFrames();
//            finalHandPose.getFramePoseIncludingFrame(handPose);
            straightLineTrajectory.compute(yoTime.getDoubleValue() - initialTime);
            straightLineTrajectory.getPose(handPose);
            straightLineTrajectory.getVelocity(desiredHandLinearVelocity);
            straightLineTrajectory.getAngularVelocity(desiredHandAngularVelocity);

//            circleTrajectory.compute(yoTime.getDoubleValue() - initialTime);
//            circleTrajectory.get(handPose);
//            circleTrajectory.packVelocity(desiredHandLinearVelocity);
//            circleTrajectory.packAngularVelocity(desiredHandAngularVelocity);

            desiredHandLinearVelocity.changeFrame(handControlFrame);
            desiredHandAngularVelocity.changeFrame(handControlFrame);
            desiredHandTwist.set(leftHandFrame, chestFrame, handControlFrame, desiredHandLinearVelocity, desiredHandAngularVelocity);

            taskspaceToJointspaceCalculator.compute(handPose, desiredHandTwist);
            taskspaceToJointspaceCalculator.getDesiredJointAnglesIntoOneDoFJoints(armJoints);
            ScrewTools.setJointPositions(armJoints, taskspaceToJointspaceCalculator.getDesiredJointAngles());
            ScrewTools.setVelocities(armJoints, taskspaceToJointspaceCalculator.getDesiredJointVelocities());
            ScrewTools.setJointAccelerations(armJoints, taskspaceToJointspaceCalculator.getDesiredJointAccelerations());

            for (OneDoFJoint joint : armJoints)
            {
               qd_fds.get(joint).update(joint.getQ());
               qdd_fds.get(joint).update(joint.getQd());
            }

            errorScalar.set(taskspaceToJointspaceCalculator.getSpatialErrorScalar());

            DenseMatrix64F spatialError = taskspaceToJointspaceCalculator.getSubspaceSpatialError();
            for (int rowIndex = 0; rowIndex < spatialError.getNumRows(); rowIndex++)
               errorSpatial[rowIndex].set(spatialError.get(rowIndex, 0));

            handPose.setToZero(handControlFrame);
            handPose.changeFrame(worldFrame);
            currentHandPose.set(handPose);

            if (!BENCHMARK)
               jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
            yoTime.add(dt);

            if (!BENCHMARK)
               scs.tickAndUpdate();
            counter++;
         }
         if (errorScalar.getDoubleValue() < 0.001)
            numberOfSuccess.increment();
      }
      if(!BENCHMARK)
         ThreadTools.sleepForever();

      endTime = System.currentTimeMillis();
      System.out.println("Total time: " + Conversions.millisecondsToSeconds(endTime - startTime));
      System.out.println("Number of iterations: " + counter);
      System.out.println("Time per iteration in millisecs: " + ((double) (endTime - startTime)) / ((double) counter));
   }

   private void applyDisturbances()
   {
      pelvisPoseWithWiggle.setIncludingFrame(initialPelvisPose);

      if (WIGGLE_PELVIS_XYZ)
      {
         double x = RandomNumbers.nextDouble(random, pelvisXYZNoise) + amplitudeWigglePelvisXYZ * Math.sin(2.0 * Math.PI * frequencyWigglePelvisXYZ * yoTime.getDoubleValue() + 0.0 / 3.0 * Math.PI);
         double y = RandomNumbers.nextDouble(random, pelvisXYZNoise) + amplitudeWigglePelvisXYZ * Math.sin(2.0 * Math.PI * frequencyWigglePelvisXYZ * yoTime.getDoubleValue() + 2.0 / 3.0 * Math.PI);
         double z = RandomNumbers.nextDouble(random, pelvisXYZNoise) + amplitudeWigglePelvisXYZ * Math.sin(2.0 * Math.PI * frequencyWigglePelvisXYZ * yoTime.getDoubleValue() + 4.0 / 3.0 * Math.PI);
         pelvisPoseWithWiggle.prependTranslation(x, y, z);
      }

      if (WIGGLE_PELVIS_YAW_PITCH_ROLL)
      {
         double yaw   = amplitudeWigglePelvisYPR * Math.sin(2.0 * Math.PI * frequencyWigglePelvisYPR * yoTime.getDoubleValue() + 1.0 / 3.0 * Math.PI);
         double pitch = amplitudeWigglePelvisYPR * Math.sin(2.0 * Math.PI * frequencyWigglePelvisYPR * yoTime.getDoubleValue() + 3.0 / 3.0 * Math.PI);
         double roll  = amplitudeWigglePelvisYPR * Math.sin(2.0 * Math.PI * frequencyWigglePelvisYPR * yoTime.getDoubleValue() + 5.0 / 3.0 * Math.PI);
         pelvisPoseWithWiggle.setOrientationYawPitchRoll(yaw, pitch, roll);
      }

      RigidBodyTransform transform = new RigidBodyTransform();
      pelvisPoseWithWiggle.get(transform);
      fullRobotModel.getRootJoint().setPositionAndRotation(transform);

      if (NOISE_ON_BACK_JOINTS)
      {
         OneDoFJoint[] backJoints = ScrewTools.createOneDoFJointPath(fullRobotModel.getPelvis(), fullRobotModel.getChest());
         double blop = 0.0;
         for (OneDoFJoint joint : backJoints)
         {
            double q = RandomNumbers.nextDouble(random, noiseAmplitudeForBackJoints);
            blop += 2.0;
            joint.setQ(q);
         }
      }
      fullRobotModel.updateFrames();
   }

   public static void main(String[] args)
   {
      new TaskspaceToJointspaceCalculatorVisualizer();
   }
}
