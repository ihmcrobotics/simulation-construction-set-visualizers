package us.ihmc.scsVisualizers.instantaneousCapturePoint;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSmoothCMPPlannerParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.FootstepAngularMomentumPredictor;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class CMPPlannerVisualizer
{
   private final double defaultSwingTime = 1.0;
   private final double defaultTransferTime = 0.5;
   private final double gravityZ = 9.81;
   private final double robotHeight = 0.9;
   private static final int BUFFER_SIZE = 16000;

   private final static int numberOfSteps = 4;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry("CoPPlanVisualizer");
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final SimulationConstructionSet scs;

   private final List<YoFramePoseUsingYawPitchRoll> yoNextFootstepPose = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> yoNextFootstepPolygon = new ArrayList<>();
   private final ArrayList<Updatable> updatables = new ArrayList<>();
   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final YoDouble yoTime;
   private final double dt = 0.006;

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());
   private final YoDouble omega0;
   private SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private MidFootZUpGroundFrame midFeetZUpFrame;
   private SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   private SideDependentList<YoFramePoseUsingYawPitchRoll> currentFootPoses = new SideDependentList<>();
   private BipedSupportPolygons bipedSupportPolygons;

   private SmoothCMPPlannerParameters planParameters;
   private ReferenceCoPTrajectoryGenerator copGenerator;
   private ReferenceCMPTrajectoryGenerator cmpGenerator;
   private FootstepAngularMomentumPredictor estAngMomGenerator;

   private FootstepTestHelper footstepTestHelper;
   private final AtlasPhysicalProperties atlasPhysicalProperties = new AtlasPhysicalProperties();
   private List<Footstep> footsteps;
   private List<FootstepTiming> footstepTimings;
   private BagOfBalls copTrack, cmpTrack, comTrack, swFootTrack;
   private BagOfBalls copWaypointViz;

   private final int simulatedTicksPerGraphicUpdate = 2;

   private List<YoDouble> swingDurations = new ArrayList<>();
   private List<YoDouble> touchdownDurations = new ArrayList<>();
   private List<YoDouble> transferDurations = new ArrayList<>();
   private List<YoDouble> swingSplitFractions = new ArrayList<>();
   private List<YoDouble> swingDurationShiftFractions = new ArrayList<>();
   private List<YoDouble> transferSplitFractions = new ArrayList<>();

   private final YoFramePoint3D desiredCoP;
   private final YoFramePoint3D desiredCMP;
   private final YoFrameVector3D desiredCoPVelocity;
   private final YoFramePoint3D predictedCoM, predictedSwingFootLocation;
   private final YoFrameVector3D predictedAngularMomentum;
   private final YoFrameVector3D predictedTorque;

   private final List<FootstepData> upcomingFootstepData = new ArrayList<>();
   private final YoInteger numberOfFootstepsRegistered;

   public CMPPlannerVisualizer()
   {
      omega0 = new YoDouble("Omega0", registry);
      omega0.set(Math.sqrt(gravityZ / robotHeight));
      setupPlanner();
      for (int i = 0; i < planParameters.getNumberOfFootstepsToConsider(); i++)
      {
         YoFrameConvexPolygon2D nextFootPolygon = new YoFrameConvexPolygon2D("nextFootstep" + i, "", worldFrame, 4, registry);
         yoNextFootstepPolygon.add(nextFootPolygon);
         graphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep" + i, nextFootPolygon, Color.blue, false));
      }
      desiredCoP = new YoFramePoint3D("desiredCoP", worldFrame, registry);
      desiredCMP = new YoFramePoint3D("desiredCMP", worldFrame, registry);
      predictedCoM = new YoFramePoint3D("predictedCoM", worldFrame, registry);
      predictedSwingFootLocation = new YoFramePoint3D("predictedSwFootLoc", worldFrame, registry);
      predictedAngularMomentum = new YoFrameVector3D("predictedAngularMomentum", worldFrame, registry);
      predictedTorque = new YoFrameVector3D("predictedTorque", worldFrame, registry);
      desiredCoPVelocity = new YoFrameVector3D("desiredCoPVelocity", worldFrame, registry);

      numberOfFootstepsRegistered = new YoInteger("numberOfFootstepsRegistered", registry);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<Point2D>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));
      for (int i = 0; i < planParameters.getNumberOfFootstepsToConsider(); i++)
      {
         YoFramePoseUsingYawPitchRoll nextPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose" + i, worldFrame, registry);
         yoNextFootstepPose.add(nextPose);
         graphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep" + i, footstepGraphics, nextPose, 1.0));
      }
      footsteps = footstepTestHelper.createFootsteps(0.25, 0.3, numberOfSteps);
      footstepTimings = getFootstepTimings();
      updatables.add(new Updatable()
      {
         @Override
         public void update(double time)
         {

            for (YoGraphicsList yoGraphicsList : graphicsListRegistry.getYoGraphicsLists())
            {
               for (YoGraphic yoGraphic : yoGraphicsList.getYoGraphics())
               {
                  yoGraphic.update();
               }
            }

            for (RobotSide robotSide : RobotSide.values)
            {
               soleFrames.get(robotSide).update();
               ankleZUpFrames.get(robotSide).update();
               soleZUpFrames.get(robotSide).update();
            }
            midFeetZUpFrame.update();
         }
      });

      copTrack = new BagOfBalls(50, 0.005, "CoP", YoAppearance.White(), registry, graphicsListRegistry);
      cmpTrack = new BagOfBalls(50, 0.005, "eCMP", YoAppearance.Purple(), registry, graphicsListRegistry);
      comTrack = new BagOfBalls(50, 0.005, "CoM", YoAppearance.Red(), registry, graphicsListRegistry);
      swFootTrack = new BagOfBalls(50, 0.005, "swFoot", YoAppearance.Blue(), registry, graphicsListRegistry);
      copWaypointViz = new BagOfBalls(50, 0.007, "CoPWaypointViz", YoAppearance.Black(), registry, graphicsListRegistry);
      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Dummy");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.5);
      scs.setCameraPosition(-0.5, 0.0, 1.0);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(graphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();
      scs.startOnAThread();
      simulate(footsteps, footstepTimings);
      ThreadTools.sleepForever();
   }

   private List<FootstepTiming> getFootstepTimings()
   {
      List<FootstepTiming> timings = new ArrayList<>(footsteps.size());
      for (int i = 0; i < footsteps.size(); i++)
      {
         timings.add(new FootstepTiming(defaultSwingTime, defaultTransferTime));
      }
      return timings;
   }

   private void simulate(List<Footstep> footsteps, List<FootstepTiming> timings)
   {
      int currentStepCount = 0;
      YoBoolean inDoubleSupport = new YoBoolean("InDoubleSupport", registry);
      inDoubleSupport.set(true);

      if (footsteps.size() != timings.size())
         return;

      List<Footstep> nextFootsteps = new ArrayList<>();
      List<FootstepTiming> nextFootstepTimings = new ArrayList<>();

      while (currentStepCount < footsteps.size() + 1)
      {
         nextFootsteps.clear();
         nextFootstepTimings.clear();
         registerFootsteps(currentStepCount);
         int numberOfNextSteps = footsteps.size() - currentStepCount < planParameters.getNumberOfFootstepsToConsider() ?
               footsteps.size() - currentStepCount :
               planParameters.getNumberOfFootstepsToConsider();
         for (int i = 0; i < numberOfNextSteps; i++)
         {
            nextFootsteps.add(footsteps.get(currentStepCount + i));
            nextFootstepTimings.add(timings.get(currentStepCount + i));
         }
         updateFootStepViz(nextFootsteps, nextFootstepTimings);

         Footstep nextFootstep = footsteps.get(footsteps.size() - 1);
         if (currentStepCount != footsteps.size())
            nextFootstep = nextFootsteps.get(0);

         if (inDoubleSupport.getBooleanValue())
         {
            for (RobotSide robotSide : RobotSide.values)
               contactStates.get(robotSide).setFullyConstrained();
         }
         else if (nextFootstep != null)
         {
            RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
            contactStates.get(supportSide.getOppositeSide()).clear();
         }
         bipedSupportPolygons.updateUsingContactStates(contactStates);

         if (inDoubleSupport.getBooleanValue())
         {
            if (currentStepCount == 0)
            {
               copGenerator.computeReferenceCoPsStartingFromDoubleSupport(true, nextFootstep.getRobotSide().getOppositeSide(), nextFootstep.getRobotSide());
               estAngMomGenerator.addCopAndComSetpointsToPlan(copGenerator.getWaypoints(), null, null, null, null, null, null,
                                                              copGenerator.getNumberOfFootstepsRegistered());
               estAngMomGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(true, true);
            }
            else if (currentStepCount == footsteps.size())
            {
               copGenerator.computeReferenceCoPsStartingFromDoubleSupport(false, nextFootstep.getRobotSide(), nextFootstep.getRobotSide().getOppositeSide());
               estAngMomGenerator.addCopAndComSetpointsToPlan(copGenerator.getWaypoints(), null, null, null, null, null, null,
                                                              copGenerator.getNumberOfFootstepsRegistered());
               estAngMomGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(false, false);
            }
            else
            {
               copGenerator.computeReferenceCoPsStartingFromDoubleSupport(false, nextFootstep.getRobotSide().getOppositeSide(), nextFootstep.getRobotSide());
               estAngMomGenerator.addCopAndComSetpointsToPlan(copGenerator.getWaypoints(), null, null, null, null, null, null,
                                                              copGenerator.getNumberOfFootstepsRegistered());
               estAngMomGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(false, false);
            }

            cmpGenerator.setNumberOfRegisteredSteps(copGenerator.getNumberOfFootstepsRegistered());
            //            copGenerator.initializeForTransfer(yoTime.getDoubleValue());
            estAngMomGenerator.initializeForDoubleSupport(yoTime.getDoubleValue(), false);
            cmpGenerator.initializeForTransfer(yoTime.getDoubleValue(), copGenerator.getTransferCoPTrajectories(), copGenerator.getSwingCoPTrajectories(),
                                               estAngMomGenerator.getTransferAngularMomentumTrajectories(),
                                               estAngMomGenerator.getSwingAngularMomentumTrajectories());
            inDoubleSupport.set(false);
         }
         else if (nextFootstep != null)
         {
            copGenerator.computeReferenceCoPsStartingFromSingleSupport(nextFootstep.getRobotSide().getOppositeSide());
            cmpGenerator.setNumberOfRegisteredSteps(copGenerator.getNumberOfFootstepsRegistered());
            estAngMomGenerator
                  .addCopAndComSetpointsToPlan(copGenerator.getWaypoints(), null, null, null, null, null, null, copGenerator.getNumberOfFootstepsRegistered());
            estAngMomGenerator.computeReferenceAngularMomentumStartingFromSingleSupport();

            copGenerator.initializeForSwing();
            estAngMomGenerator.initializeForSingleSupport(yoTime.getDoubleValue());
            cmpGenerator.initializeForSwing(yoTime.getDoubleValue(), copGenerator.getTransferCoPTrajectories(), copGenerator.getSwingCoPTrajectories(),
                                            estAngMomGenerator.getTransferAngularMomentumTrajectories(),
                                            estAngMomGenerator.getSwingAngularMomentumTrajectories());
            inDoubleSupport.set(true);
            FootSpoof footSpoof = contactableFeet.get(nextFootstep.getRobotSide());
            FramePose3D nextSupportPose = footPosesAtTouchdown.get(nextFootstep.getRobotSide());
            nextFootstep.getPose(nextSupportPose);
            nextSupportPose.changeFrame(worldFrame);
            footSpoof.setSoleFrame(nextSupportPose);
         }

         /*
          * if (nextFootstep.getPredictedContactPoints() == null)
          * contactStates.get(nextFootstep.getRobotSide()).setContactFramePoints
          * (footSpoof.getContactPoints2d()); else
          * contactStates.get(nextFootstep.getRobotSide()).setContactPoints(
          * nextFootstep.getPredictedContactPoints());
          * bipedSupportPolygons.updateUsingContactStates(contactStates);
          */

         List<CoPPointsInFoot> copList = copGenerator.getWaypoints();
         copWaypointViz.reset();
         for (int i = 0; i < copList.size(); i++)
         {
            CoPPointsInFoot copPoints = copList.get(i);
            for (int j = 0; j < copPoints.getCoPPointList().size(); j++)
            {
               FramePoint3D tempPoint = new FramePoint3D(copPoints.getWaypointInWorld(j));
               tempPoint.add(0.0, 0.0, 0.05);
               copWaypointViz.setBall(tempPoint);
            }
         }
         double totalTime = transferDurations.get(0).getDoubleValue();
         if (currentStepCount != footsteps.size())
            totalTime = inDoubleSupport.getBooleanValue() ? nextFootstepTimings.get(0).getSwingTime() : nextFootstepTimings.get(0).getTransferTime();
         for (double time = 0.0; time < totalTime; time += dt)
         {
            simulateOneTick();
         }

         copGenerator.updateListeners();

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();
         if (currentStepCount == footsteps.size())
            break;
         currentStepCount = inDoubleSupport.getBooleanValue() ? currentStepCount + 1 : currentStepCount;
      }
   }

   private int counter = 0;

   private void simulateOneTick()
   {
      callUpdatables();
      updateFootViz();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      copGenerator.update(yoTime.getDoubleValue());
      estAngMomGenerator.update(yoTime.getDoubleValue());
      cmpGenerator.update(yoTime.getDoubleValue());

      copGenerator.getDesiredCenterOfPressure(desiredCoP, desiredCoPVelocity);
      estAngMomGenerator.getPredictedCenterOfMassPosition(predictedCoM, yoTime.getDoubleValue());
      estAngMomGenerator.getPredictedFootPosition(predictedSwingFootLocation, yoTime.getDoubleValue());
      estAngMomGenerator.getDesiredAngularMomentum(predictedAngularMomentum, predictedTorque);

      cmpGenerator.getPosition(desiredCMP);

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         FramePoint3D desiredCoP = new FramePoint3D(this.desiredCoP);
         desiredCoP.setZ(0.05);
         copTrack.setBallLoop(desiredCoP);

         FramePoint3D desiredCMP = new FramePoint3D(this.desiredCMP);
         desiredCMP.setZ(0.05);
         cmpTrack.setBallLoop(desiredCMP);

         FramePoint3D predictedCoM = new FramePoint3D(this.predictedCoM);
         predictedCoM.setZ(0.05);
         comTrack.setBallLoop(predictedCoM);

         FramePoint3D predictedSwFootLoc = new FramePoint3D(this.predictedSwingFootLocation);
         swFootTrack.setBallLoop(predictedSwFootLoc);
      }
      yoTime.add(dt);
      scs.tickAndUpdate();
   }

   private void callUpdatables()
   {
      for (Updatable updatable : updatables)
         updatable.update(yoTime.getDoubleValue());
   }

   private void registerFootsteps(int index)
   {
      copGenerator.clear();
      estAngMomGenerator.clear();
      cmpGenerator.reset();
      upcomingFootstepData.clear();
      numberOfFootstepsRegistered.set(0);
      for (int i = index; i < footsteps.size(); i++)
      {
         numberOfFootstepsRegistered.increment();
         upcomingFootstepData.add(new FootstepData(footsteps.get(i), footstepTimings.get(i)));
      }
   }

   private void updateFootViz()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (contactStates.get(robotSide).inContact())
         {
            FramePose3D footPose = new FramePose3D(contactableFeet.get(robotSide).getSoleFrame());
            footPose.changeFrame(worldFrame);
            currentFootPoses.get(robotSide).set(footPose);
         }
         else
         {
            currentFootPoses.get(robotSide).setToNaN();
         }
      }
   }

   private FrameConvexPolygon2D tempFootPolygon = new FrameConvexPolygon2D();

   private void updateFootStepViz(List<Footstep> nextFootsteps, List<FootstepTiming> nextFootstepTimings)
   {
      if (nextFootsteps.size() != nextFootstepTimings.size())
      {
         setVizFootstepsToNaN();
         return;
      }

      int numberOfPolygonsToUpdate = nextFootsteps.size() < yoNextFootstepPose.size() ? nextFootsteps.size() : yoNextFootstepPolygon.size();
      for (int i = 0; i < numberOfPolygonsToUpdate; i++)
      {
         Footstep footstep = nextFootsteps.get(i);
         if (footstep.getPredictedContactPoints() == null)
         {
            footstep.setPredictedContactPoints(contactableFeet.get(footstep.getRobotSide()).getContactPoints2d());
         }
         ReferenceFrame footFrame = ReferenceFrame
               .constructFrameWithUnchangingTranslationFromParent("FootstepFrame", worldFrame, footstep.getFootstepPose().getPosition());
         tempFootPolygon.setIncludingFrame(footFrame, Vertex2DSupplier.asVertex2DSupplier(footstep.getPredictedContactPoints()));
         tempFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
         yoNextFootstepPolygon.get(i).set(tempFootPolygon);
         yoNextFootstepPose.get(i).setMatchingFrame(new FramePose3D(footFrame));
      }
      for (int i = numberOfPolygonsToUpdate; i < yoNextFootstepPose.size(); i++)
         setVizFootstepsToNaN(i);
   }

   private void setVizFootstepsToNaN(int index)
   {
      yoNextFootstepPose.get(index).setToNaN();
      yoNextFootstepPolygon.get(index).clear();
   }

   private void setVizFootstepsToNaN()
   {
      for (int i = 0; i < yoNextFootstepPose.size(); i++)
         setVizFootstepsToNaN(i);
   }

   private void setupPlanner()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
         contactPointsInSoleFrame
               .add(new Point2D(atlasPhysicalProperties.getFootLengthForControl() / 2.0, atlasPhysicalProperties.getToeWidthForControl() / 2.0));
         contactPointsInSoleFrame
               .add(new Point2D(atlasPhysicalProperties.getFootLengthForControl() / 2.0, -atlasPhysicalProperties.getToeWidthForControl() / 2.0));
         contactPointsInSoleFrame
               .add(new Point2D(-atlasPhysicalProperties.getFootLengthForControl() / 2.0, -atlasPhysicalProperties.getFootWidthForControl() / 2.0));
         contactPointsInSoleFrame
               .add(new Point2D(-atlasPhysicalProperties.getFootLengthForControl() / 2.0, atlasPhysicalProperties.getFootWidthForControl() / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = new FramePose3D(worldFrame);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);
         currentFootPoses.put(robotSide, new YoFramePoseUsingYawPitchRoll(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         graphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));

         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction,
                                                                           registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);

         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "AnkleZUp"));
         soleFrames.put(robotSide, soleFrame);
         soleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, soleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "SoleZUp"));
      }
      midFeetZUpFrame = new MidFootZUpGroundFrame("MidFeetZUpFrame", soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, registry, graphicsListRegistry);
      footstepTestHelper = new FootstepTestHelper(contactableFeet);
      planParameters = new AtlasSmoothCMPPlannerParameters(atlasPhysicalProperties)
      {
         @Override
         public boolean planSwingAngularMomentum()
         {
            return true;
         }

         @Override
         public boolean planTransferAngularMomentum()
         {
            return true;
         }
      };
      YoInteger numberOfFootstepsToConsider = new YoInteger("numberOfFootstepsToConsider", registry);
      numberOfFootstepsToConsider.set(planParameters.getNumberOfFootstepsToConsider());

      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         YoDouble swingDuration = new YoDouble("swingDuration" + i, registry);
         YoDouble transferDuration = new YoDouble("transferDuration" + i, registry);
         YoDouble touchdownDuration = new YoDouble("touchdownDuration" + i, registry);
         swingDuration.set(defaultSwingTime);
         transferDuration.set(defaultTransferTime);
         swingDurations.add(swingDuration);
         transferDurations.add(transferDuration);
         touchdownDurations.add(touchdownDuration);

         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, registry);
         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         swingSplitFraction.set(planParameters.getSwingSplitFraction());
         swingDurationShiftFraction.set(planParameters.getSwingDurationShiftFraction());
         transferSplitFraction.set(planParameters.getTransferSplitFraction());
         swingSplitFractions.add(swingSplitFraction);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
         transferSplitFractions.add(transferSplitFraction);
      }
      int i = numberOfFootstepsToConsider.getIntegerValue();
      YoDouble transferDuration = new YoDouble("transferDuration" + i, registry);
      YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
      transferDuration.set(defaultTransferTime);
      transferSplitFraction.set(planParameters.getTransferSplitFraction());
      transferDurations.add(transferDuration);
      transferSplitFractions.add(transferSplitFraction);

      String namePrefix = "TestCMPPlanner";
      int maxNumberFootstepsToConsider = planParameters.getNumberOfFootstepsToConsider();
      copGenerator = new ReferenceCoPTrajectoryGenerator(namePrefix, maxNumberFootstepsToConsider, bipedSupportPolygons, contactableFeet,
                                                         numberOfFootstepsToConsider, swingDurations, transferDurations, touchdownDurations,
                                                         swingSplitFractions, swingDurationShiftFractions, transferSplitFractions, numberOfFootstepsRegistered,
                                                         upcomingFootstepData, registry);
      cmpGenerator = new ReferenceCMPTrajectoryGenerator(namePrefix, maxNumberFootstepsToConsider, numberOfFootstepsToConsider, false, registry,
                                                         graphicsListRegistry);
      cmpGenerator.setGroundReaction(180 * gravityZ);
      estAngMomGenerator = new FootstepAngularMomentumPredictor(namePrefix, omega0, maxNumberFootstepsToConsider, registry);
      FullHumanoidRobotModel fullHumanoidRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
            .createFullRobotModel();
      estAngMomGenerator.initializeParameters(planParameters, fullHumanoidRobotModel.getTotalMass(), gravityZ);
      copGenerator.initializeParameters(planParameters);

      YoGraphicsList yoGraphicsList = new YoGraphicsList("graphicsList");
      ArtifactList artifactList = new ArtifactList("artifactList");
      copGenerator.createVisualizerForConstantCoPs(yoGraphicsList, artifactList);

      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      graphicsListRegistry.registerArtifactList(artifactList);
   }

   public static void main(String[] args)
   {
      new CMPPlannerVisualizer();
   }

}
