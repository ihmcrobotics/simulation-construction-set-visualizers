package us.ihmc.scsVisualizers.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.jointAnglesWriter.JointAnglesWriter;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class CutForceControllerVisualizer
{
   /**
    * Simulation and SCS parameters and variables
    */

   private static final double TRAJECTORYTIME = 15.0;
   private static final int NUMBEROFTRAJECTORIES = 5; // length of trajectory arraylist

   private static final int BUFFERSIZE = 10000;
   private static final double DTCONTROL = 0.004;

   private static final int numberOfIterations = BUFFERSIZE / NUMBEROFTRAJECTORIES - 1;

   private AtlasRobotModel atlasRobotModel;
   private FullHumanoidRobotModel fullRobotModel;
   private HumanoidFloatingRootJointRobot robot;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private SimulationConstructionSet scs;
   private SimulationConstructionSetParameters scsParameters;
   private final YoDouble yoTime;

   private final FramePose3D initialPelvisPose = new FramePose3D();
//   private final RobotSide robotSide = RobotSide.RIGHT;

   /**
    * Trajectory stuff
    */
   private Twist desiredHandTwist = new Twist();
   private final FrameVector3D desiredHandLinearVelocity = new FrameVector3D();
   private final FrameVector3D desiredHandAngularVelocity = new FrameVector3D();

   private TaskspaceToJointspaceCalculator rightHandTaskTojointSpaceCalculator;
   private JointAnglesWriter jointAnglesWriter;
   private OneDoFJoint[] rightArmJoints;

   private StraightLinePoseTrajectoryGenerator straightLineTrajectory;

   private static final ReferenceFrame worldFrame = HumanoidReferenceFrames.getWorldFrame();
   private ReferenceFrame rightHandControlFrame;
   private ReferenceFrame leftHandControlFrame;
   private ReferenceFrame rightWristFrame;
   private ReferenceFrame leftWristFrame;

   private final YoFramePoseUsingYawPitchRoll finalHandPose = new YoFramePoseUsingYawPitchRoll("finalHandPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll initialHandPose = new YoFramePoseUsingYawPitchRoll("initialHandPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll currentHandPose = new YoFramePoseUsingYawPitchRoll("currentHandPose", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll rightWristSensorCoordinateSystemPose = new YoFramePoseUsingYawPitchRoll("rightWristSensorCoordinateSystem", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll leftWristSensorCoordinateSystemPose = new YoFramePoseUsingYawPitchRoll("leftWristSensorCoordinateSystem", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll leftHandControlCoordinateSystemPose = new YoFramePoseUsingYawPitchRoll("leftHandControlFrameCoordinateSystem", worldFrame, registry);
   
   private final FramePose3D handPose = new FramePose3D();

   private final ArrayList<FramePose3D> wayPoints = new ArrayList<FramePose3D>();

   /**
    * ForceStuff
    */
   private Wrench simulatedWristWrench;
   private final YoDouble simulatedFx = new YoDouble("simulatedFx", registry);
   private final YoDouble simulatedFy = new YoDouble("simulatedFy", registry);
   private final YoDouble simulatedFz = new YoDouble("simulatedFz", registry);

   private final FrameVector3D simulatedForceVector = new FrameVector3D();

   private YoDouble coeffC1 = new YoDouble("coeffC1", registry);
   private YoDouble coeffC2 = new YoDouble("coeffC2", registry);
   
   private final YoDouble quadraticForcecoeff = new YoDouble("quadraticcoeff", registry);
   

   private final YoDouble scaledVelocity = new YoDouble("scaledVelocity", registry);
   private final YoDouble scaledTimeVariable = new YoDouble("scaledTime", registry);

   private final static double WALLX = 0.6;

   /**
    * Noise
    */
   private final double maxNoiseAmp = 0.1;
   private final Random random = new Random(23156489463L);

   /**
    * ForceController and MPA
    */
   private final double DESIRED_FTANG = -10.0;
   private YoDouble currentFTang = new YoDouble("currentFTangential", registry);
   private YoDouble currentFTangModel = new YoDouble("currentFTangModel", registry);
   private YoDouble currentTangentialVelocity = new YoDouble("currentTangentialVelocity", registry);

   private double preScalingTrajectoryVelocityDouble;

   private final FramePoint3D preScalingTrajectoryPosition = new FramePoint3D(worldFrame);
   private final FrameVector3D preScalingTrajectoryVelocity = new FrameVector3D(worldFrame);
   private final FrameVector3D preScalingTrajectoryAcceleration = new FrameVector3D(worldFrame);



   private final YoDouble vMPC = new YoDouble("vMPC", registry);

   double lnC1, c2, epsilon;
   double f1 = 1.0;
   double f2 = 8.0;

   private final static double MAXSCALE = 5.0;


   //if the model is not conservative enough it leads to a severe overshoot in deltaf--> limit maximal possible timescaling.

//   private double pMPC = 0.0013;
//   private double iMPC = 0.00035;
//   private double dMPC = 0.00001;
   private double oldDeltaF;
   private YoDouble deltaF = new YoDouble("deltaF", registry);
   private YoDouble deltaFIntegrator = new YoDouble("deltaFIntegrator", registry);
   private YoDouble deltaFDerivative = new YoDouble("deltaFDerivative", registry);

   private YoDouble scaleFactor = new YoDouble("scaleFactor", registry);

   private Vector3D tempVector = new Vector3D();
   private Vector3D tangentVectorInWorld = new Vector3D();
   private Point3D nextPositionInWorld = new Point3D();

   private Point3D currentPositionInWorld = new Point3D();
   private FramePose3D tempPose = new FramePose3D();


   public CutForceControllerVisualizer()
   {
      quadraticForcecoeff.set(1100.0);

      atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      fullRobotModel = atlasRobotModel.createFullRobotModel();
      robot = atlasRobotModel.createHumanoidFloatingRootJointRobot(false);
      atlasRobotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeRobot(robot, atlasRobotModel.getJointMap());

      SDFPerfectSimulatedSensorReader sensorReader = new SDFPerfectSimulatedSensorReader(robot, fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));
      sensorReader.read();

      fullRobotModel.updateFrames();
      initialPelvisPose.setToZero(fullRobotModel.getRootJoint().getFrameAfterJoint());
      initialPelvisPose.changeFrame(worldFrame);

      scsParameters = new SimulationConstructionSetParameters(true, BUFFERSIZE);
      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.setPlaybackRealTimeRate(1.0);
      scs.setDT(DTCONTROL, 1);


      RotationMatrix zRotationMatrix = new RotationMatrix();
      zRotationMatrix.setToYawMatrix(Math.PI / 2.0);
      Quaternion orientation = new Quaternion(zRotationMatrix);
      // The Quat4d.set(Matrix3d) is buggy
//      orientation.set(zRotationMatrix);

      wayPoints.add(new FramePose3D(worldFrame, new Point3D(WALLX, -0.2, 1.0), orientation));
      wayPoints.add(new FramePose3D(worldFrame, new Point3D(WALLX,  0.2, 1.0), orientation));
      wayPoints.add(new FramePose3D(worldFrame, new Point3D(WALLX,  0.2, 1.4), orientation));
      wayPoints.add(new FramePose3D(worldFrame, new Point3D(WALLX, -0.2, 1.4), orientation));
      wayPoints.add(new FramePose3D(worldFrame, new Point3D(WALLX, -0.2, 1.0), orientation));




      RigidBody chest = fullRobotModel.getChest();
      RigidBody rightHand = fullRobotModel.getHand(RobotSide.RIGHT);
      ReferenceFrame rightHandFrame = rightHand.getBodyFixedFrame();
      ReferenceFrame chestFrame = chest.getBodyFixedFrame();
      rightHandControlFrame = fullRobotModel.getHandControlFrame(RobotSide.RIGHT);
      leftHandControlFrame = fullRobotModel.getHandControlFrame(RobotSide.LEFT);
      


      straightLineTrajectory = new StraightLinePoseTrajectoryGenerator("straightLineTrajectory", true, worldFrame, registry, true, yoGraphicsListRegistry);
      straightLineTrajectory.showVisualization();
      //    straightLineTrajectory.registerNewTrajectoryFrame(chestFrame);
      straightLineTrajectory.setTrajectoryTime(TRAJECTORYTIME);




      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("initialPose", initialHandPose, 0.1, YoAppearance.Blue()));
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("currentPose", currentHandPose, 0.3, YoAppearance.Red()));
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("desiredPose", finalHandPose, 0.1, YoAppearance.Green()));
      
      
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("rightWristsensorFrame", rightWristSensorCoordinateSystemPose, 0.3, YoAppearance.Black()));
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("leftWristsensorFrame", leftWristSensorCoordinateSystemPose, 0.3, YoAppearance.Black()));
      yoGraphicsListRegistry.registerYoGraphic("Viz", new YoGraphicCoordinateSystem("leftHandControlFrame", leftHandControlCoordinateSystemPose, 0.3, YoAppearance.Black()));
      
      
      
      
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.addYoVariableRegistry(registry);


      rightHandTaskTojointSpaceCalculator = new TaskspaceToJointspaceCalculator("rightHand", chest, rightHand, DTCONTROL, registry);
      rightHandTaskTojointSpaceCalculator.setupWithDefaultParameters();
      rightHandTaskTojointSpaceCalculator.setControlFrameFixedInEndEffector(rightHandControlFrame);
      jointAnglesWriter =  new JointAnglesWriter(robot, fullRobotModel);
      



      rightArmJoints = ScrewTools.createOneDoFJointPath(chest, rightHand);

      scs.startOnAThread();
      scs.tickAndUpdate();
      yoTime = robot.getYoTime();
      scaleFactor.set(1.0);

      // initialize force model coefficients
      coeffC1.set(90.0); //45.0
      coeffC2.set(0.5); // 1.2

      lnC1 = Math.log(coeffC1.getDoubleValue());
      c2 = coeffC2.getDoubleValue();



      /**
       * Simulation of Wrist sensors
       */
            
      ArrayList<WrenchCalculatorInterface> forceSensors = new ArrayList<WrenchCalculatorInterface>();
      robot.getForceSensors(forceSensors);
      SideDependentList<WrenchCalculatorInterface> wristSCSSensors = new SideDependentList<>();

      for (int i = 0; i < forceSensors.size(); i++)
      {
         if (forceSensors.get(i).getName().contains("l_arm_wry2"))
            wristSCSSensors.put(RobotSide.LEFT, forceSensors.get(i));
         else if (forceSensors.get(i).getName().contains("r_arm_wry2"))
            wristSCSSensors.put(RobotSide.RIGHT, forceSensors.get(i));
      }

      SideDependentList<ForceSensorDefinition> wristSensorDefinitions = new SideDependentList<ForceSensorDefinition>();
      for (RobotSide robotSide : RobotSide.values)
      {
         WrenchCalculatorInterface scsSensor = wristSCSSensors.get(robotSide);
         String sensorName = scsSensor.getName();
         RigidBody rigidBody = fullRobotModel.getOneDoFJointByName(scsSensor.getJoint().getName()).getSuccessor();
         RigidBodyTransform transformFromSensorToParentJoint = new RigidBodyTransform();
         scsSensor.getTransformToParentJoint(transformFromSensorToParentJoint);
         ForceSensorDefinition forceSensorDefinition = new ForceSensorDefinition(sensorName, rigidBody, transformFromSensorToParentJoint);
         wristSensorDefinitions.put(robotSide, forceSensorDefinition);
      }

      //            final SideDependentList<ForceSensorData> wristSensors = new SideDependentList<ForceSensorData>();
      //            
      //            for (RobotSide robotSide : RobotSide.values)
      //            {
      //               ForceSensorData forceSensorData = new ForceSensorData(wristSensorDefinitions.get(robotSide));
      //               wristSensors.put(robotSide, forceSensorData);
      //            }
      //            
      rightWristFrame = wristSensorDefinitions.get(RobotSide.RIGHT).getSensorFrame();
      leftWristFrame = wristSensorDefinitions.get(RobotSide.LEFT).getSensorFrame();
      /**
       * 
       */

      simulatedWristWrench = new Wrench(rightWristFrame, rightWristFrame);

      for(int i = 0; i < NUMBEROFTRAJECTORIES; i++)
      {
         fullRobotModel.updateFrames();
         rightHandTaskTojointSpaceCalculator.initializeFromCurrentJointAngles();

         handPose.setToZero(rightHandControlFrame);
         handPose.changeFrame(worldFrame);
         currentHandPose.set(handPose);
         initialHandPose.set(handPose);

         straightLineTrajectory.switchTrajectoryFrame(worldFrame);
         straightLineTrajectory.setInitialPose(handPose);

         handPose.setIncludingFrame(wayPoints.get(i));
         finalHandPose.set(handPose);

         straightLineTrajectory.setFinalPose(handPose);
         straightLineTrajectory.initialize();
         scaledTimeVariable.set(0.0);
         deltaFIntegrator.set(0.0);
         deltaFDerivative.set(0.0);
         deltaF.set(0.0 - DESIRED_FTANG);
         oldDeltaF = 0.0;


         scs.tickAndUpdate();
         for(int j = 0; j < numberOfIterations; j++)
         {
            fullRobotModel.updateFrames();
            /**
             *  Simulate Force on wrist sensors based on heuristic cut force model
             */

            desiredHandLinearVelocity.changeFrame(worldFrame);

            // x direction no cut force but a stiff spring.
            if(currentHandPose.getPosition().getX() > WALLX)
            {
               simulatedFx.set(-(currentHandPose.getPosition().getX() - WALLX) * 1000.0);
            }
            else
            {
               simulatedFx.set(0.0);
            }
            // for real controller: subtract hand + drill mass from z coordinate
            //            simulatedFy.set(exponentialCutForceModel(desiredHandLinearVelocity).getY());
            //            simulatedFz.set(exponentialCutForceModel(desiredHandLinearVelocity).getZ());
            simulatedFy.set(quadraticCutForceModel(desiredHandLinearVelocity).getY());
            simulatedFz.set(quadraticCutForceModel(desiredHandLinearVelocity).getZ());


            addSensorNoise();
            simulatedWristWrench.setToZero(worldFrame);
            simulatedWristWrench.setLinearPartX(simulatedFx.getDoubleValue());
            simulatedWristWrench.setLinearPartY(simulatedFy.getDoubleValue());
            simulatedWristWrench.setLinearPartZ(simulatedFz.getDoubleValue());

            simulatedWristWrench.changeFrame(rightWristFrame);

            /**
             * Get Tangent Vector
             * */
            // here: no filtering since force is very smooth
            if(scaledTimeVariable.getDoubleValue() < TRAJECTORYTIME)
            {
               straightLineTrajectory.compute(scaledTimeVariable.getDoubleValue() + DTCONTROL);
               straightLineTrajectory.getPose(tempPose);
               straightLineTrajectory.getLinearData(preScalingTrajectoryPosition, preScalingTrajectoryVelocity, preScalingTrajectoryAcceleration);
               preScalingTrajectoryVelocityDouble = preScalingTrajectoryVelocity.length();

               if(tempPose.getReferenceFrame() != worldFrame)
               {
                  tempPose.changeFrame(worldFrame);
               }

               nextPositionInWorld.set(tempPose.getPosition());
               straightLineTrajectory.compute(scaledTimeVariable.getDoubleValue());
               straightLineTrajectory.getPose(tempPose);
               if(tempPose.getReferenceFrame() != worldFrame)
               {
                  tempPose.changeFrame(worldFrame);
               }
               currentPositionInWorld.set(tempPose.getPosition());
               tangentVectorInWorld.sub(nextPositionInWorld, currentPositionInWorld);
               if(tangentVectorInWorld.length() > 0.0)
               {
                  tangentVectorInWorld.normalize();
               }
               else
               {
                  System.out.println("no valid tangent vector");
               }
               /**
                * 
                */
               
               
               tempVector.set(simulatedFx.getDoubleValue(), simulatedFy.getDoubleValue(), simulatedFz.getDoubleValue());
               currentFTang.set(tempVector.dot(tangentVectorInWorld));
               oldDeltaF = deltaF.getDoubleValue(); 
               deltaF.set(currentFTang.getDoubleValue() - DESIRED_FTANG);
               deltaFDerivative.set((deltaF.getDoubleValue() - oldDeltaF) / (DTCONTROL * scaleFactor.getDoubleValue()));
               
               
               // TODO: Maybe ARW
               deltaFIntegrator.add(deltaF.getDoubleValue());

               currentTangentialVelocity.set(Math.sqrt(Math.pow(desiredHandLinearVelocity.getY(), 2)  + Math.pow(desiredHandLinearVelocity.getZ(), 2)));

               vMPC.set(1.0);

               /**
                * MPA
                */
               if(tangentVectorInWorld.length() == 1 && currentFTang.getDoubleValue() < 0.0)
               {
                  currentFTangModel.set(coeffC1.getDoubleValue() * (Math.exp(coeffC2.getDoubleValue() * currentTangentialVelocity.getDoubleValue()) - 1.0));
                  // During cutting we have always negative values. Need to take abs() because of log.
                  epsilon = Math.log(Math.abs(currentFTang.getDoubleValue()) + coeffC1.getDoubleValue()) - Math.log(currentFTangModel.getDoubleValue() + coeffC1.getDoubleValue());
                  lnC1 += f1 * epsilon;
                  c2 += f2 * c2 * epsilon;
                  coeffC1.set(Math.exp(lnC1));
                  coeffC2.set(c2);
               }   

               vMPC.set(1.0 / coeffC2.getDoubleValue() * Math.log(Math.abs(DESIRED_FTANG) / coeffC1.getDoubleValue() + 1.0));
               
               
               // If necessary, add control
               //vMPC.add(pMPC * deltaF.getDoubleValue() + iMPC * deltaFIntegrator.getDoubleValue() + dMPC * deltaFDerivative.getDoubleValue());



               if(preScalingTrajectoryVelocityDouble == 0)
               {
                  // Either beginning or end of Trajectory, scale factor = 1.0
                  scaleFactor.set(1.0);
               }
               else
               {
                  scaleFactor.set(vMPC.getDoubleValue() / preScalingTrajectoryVelocityDouble);
               }
               // Check if scaled too much:
               if(scaleFactor.getDoubleValue() > MAXSCALE)
               {
                  scaleFactor.set(MAXSCALE);
               }
               else if(scaleFactor.getDoubleValue() < 1.0 / MAXSCALE)
               {
                  scaleFactor.set(1.0 / MAXSCALE);
               }
            }
            else
            {
               // finished Trajectory. No Force control
               scaleFactor.set(1.0);
            }

            scaledTimeVariable.add(DTCONTROL * scaleFactor.getDoubleValue());
            straightLineTrajectory.compute(scaledTimeVariable.getDoubleValue());

            straightLineTrajectory.getPose(handPose);
            straightLineTrajectory.getVelocity(desiredHandLinearVelocity);
            straightLineTrajectory.getAngularVelocity(desiredHandAngularVelocity);

            // scale the velocities with the scalefFactor
            desiredHandLinearVelocity.scale(scaleFactor.getDoubleValue());
            scaledVelocity.set(desiredHandLinearVelocity.length());
            desiredHandLinearVelocity.changeFrame(rightHandControlFrame);
            desiredHandAngularVelocity.scale(scaleFactor.getDoubleValue());
            desiredHandAngularVelocity.changeFrame(rightHandControlFrame);
            desiredHandTwist.set(rightHandFrame, chestFrame, rightHandControlFrame, desiredHandLinearVelocity, desiredHandAngularVelocity);

            rightHandTaskTojointSpaceCalculator.compute(handPose, desiredHandTwist);
            rightHandTaskTojointSpaceCalculator.getDesiredJointAnglesIntoOneDoFJoints(rightArmJoints);
            ScrewTools.setJointPositions(rightArmJoints, rightHandTaskTojointSpaceCalculator.getDesiredJointAngles());
            ScrewTools.setVelocities(rightArmJoints, rightHandTaskTojointSpaceCalculator.getDesiredJointVelocities());
            ScrewTools.setJointAccelerations(rightArmJoints, rightHandTaskTojointSpaceCalculator.getDesiredJointAccelerations());

            // update the visualization of currentFramepose
            handPose.setToZero(rightHandControlFrame);
            handPose.changeFrame(worldFrame);
            currentHandPose.set(handPose);
            handPose.setToZero(leftHandControlFrame);
            handPose.changeFrame(worldFrame);
            leftHandControlCoordinateSystemPose.set(handPose);
            
            // update the visualization of the wristSensorFrames
            handPose.setToZero(rightWristFrame);
            handPose.changeFrame(worldFrame);
            rightWristSensorCoordinateSystemPose.set(handPose);
           
            handPose.setToZero(leftWristFrame);
            handPose.changeFrame(worldFrame);
            leftWristSensorCoordinateSystemPose.set(handPose);
            
            

            jointAnglesWriter.updateRobotConfigurationBasedOnFullRobotModel();
            scs.tickAndUpdate();

            yoTime.add(DTCONTROL);
         }

      }

      ThreadTools.sleepForever();
   }

   private FrameVector3D exponentialCutForceModel(FrameVector3D linearVelocity)
   {
      if (linearVelocity.getReferenceFrame() != worldFrame)
      {
         linearVelocity.changeFrame(worldFrame);
      }
      simulatedForceVector.setToZero(worldFrame);

      simulatedForceVector.setY((-1) * Math.signum(linearVelocity.getY()) * (Math.exp(Math.abs(1.2 * linearVelocity.getY())) - 1.0) * 45.0);
      simulatedForceVector.setZ((-1) * Math.signum(linearVelocity.getZ()) * (Math.exp(Math.abs(1.2 * linearVelocity.getZ())) - 1.0) * 45.0);
      return simulatedForceVector;
   }

   private FrameVector3D quadraticCutForceModel(FrameVector3D linearVelocity)
   {
      if (linearVelocity.getReferenceFrame() != worldFrame)
      {
         linearVelocity.changeFrame(worldFrame);
      }
      simulatedForceVector.setToZero(worldFrame);


      simulatedForceVector.setY((-1) * Math.signum(linearVelocity.getY()) * quadraticForcecoeff.getDoubleValue() * Math.pow(linearVelocity.getY(), 2));
      simulatedForceVector.setZ((-1) * Math.signum(linearVelocity.getZ()) * quadraticForcecoeff.getDoubleValue() * Math.pow(linearVelocity.getZ(), 2));

      return simulatedForceVector;
   }




   public void addSensorNoise()
   {
      simulatedFx.add(RandomNumbers.nextDouble(random, maxNoiseAmp));
      simulatedFy.add(RandomNumbers.nextDouble(random, maxNoiseAmp) + 2.0 * Math.sin(scaledTimeVariable.getDoubleValue() * Math.PI / 2.5));
      simulatedFz.add(RandomNumbers.nextDouble(random, maxNoiseAmp) + 2.0 * Math.sin(scaledTimeVariable.getDoubleValue() * Math.PI / 2.5));
   }

   public static void main(String[] args)
   {
      new CutForceControllerVisualizer();
   }

}
