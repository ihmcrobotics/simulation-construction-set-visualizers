package us.ihmc.scsVisualizers.instantaneousCapturePoint;

import java.awt.Color;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.ContinuousCMPBasedICPPlanner;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ICPPlanTimingVisualizer
{
   private static final int numberOfFootstepsToCreate = 4;

   private static final double footLengthForControl = 0.22;
   private static final double footWidthForControl = 0.11;
   private static final double toeWidthForControl = 0.0825;

   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private static final double initialTransferDuration = 1.0;
   private static final double transferDuration = 0.25;
   private static final double swingDuration = 0.75;

   private static final double stepWidth = 0.25;
   private static final double stepLength = 0.4;

   private final YoVariableRegistry registry;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private ReferenceFrame midFeetZUpFrame;

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<YoFramePose> currentFootPoses = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();


   private final YoDouble yoTime;

   private final ArrayList<Footstep> plannedFootsteps = new ArrayList<>();

   private final YoFramePose yoNextFootstepPlan;
   private final YoFramePose yoNextNextFootstepPlan;
   private final YoFramePose yoNextNextNextFootstepPlan;

   private final YoFramePose yoNextFootstepPose;
   private final YoFramePose yoNextNextFootstepPose;
   private final YoFramePose yoNextNextNextFootstepPose;
   private final YoFrameConvexPolygon2d yoNextFootstepPolygon;
   private final YoFrameConvexPolygon2d yoNextNextFootstepPolygon;
   private final YoFrameConvexPolygon2d yoNextNextNextFootstepPolygon;

   private BipedSupportPolygons bipedSupportPolygons;
   private FootstepTestHelper footstepTestHelper;

   private final YoDouble currentTransferInitialDuration;
   private final YoDouble currentTransferEndDuration;
   private final YoDouble swingInitialDuration;
   private final YoDouble swingEndDuration;
   private final YoDouble nextTransferInitialDuration;
   private final YoDouble nextTransferEndDuration;

   private final ContinuousCMPBasedICPPlanner icpPlanner;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public ICPPlanTimingVisualizer()
   {
      Robot robot = new DummyRobot();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      registry = robot.getRobotsYoVariableRegistry();

      ICPPlannerParameters capturePointPlannerParameters = createICPPlannerParameters();

      yoTime = robot.getYoTime();

      setupFeetFrames(yoGraphicsListRegistry);

      icpPlanner = new ContinuousCMPBasedICPPlanner(bipedSupportPolygons, contactableFeet, capturePointPlannerParameters.getNumberOfFootstepsToConsider(),
                                                    registry, yoGraphicsListRegistry);
      icpPlanner.initializeParameters(capturePointPlannerParameters);
      icpPlanner.setOmega0(3.0);
      icpPlanner.setFinalTransferDuration(1.0);

      RobotSide currentSide = RobotSide.LEFT;
      for (int i = 0; i < numberOfFootstepsToCreate; i++)
      {
         currentSide = currentSide.getOppositeSide();
         plannedFootsteps.add(new Footstep(currentSide));
      }

      currentTransferInitialDuration = new YoDouble("currentTransferInitialDuration", registry);
      currentTransferEndDuration = new YoDouble("currentTransferEndDuration", registry);
      swingInitialDuration = new YoDouble("swingInitialDuration", registry);
      swingEndDuration = new YoDouble("swingEndDuration", registry);
      nextTransferInitialDuration = new YoDouble("nextTransferInitialDuration", registry);
      nextTransferEndDuration = new YoDouble("nextTransferEndDuration", registry);

      currentTransferInitialDuration.set(0.5 * 1.0);
      currentTransferEndDuration.set(0.5 * 1.0);
      swingInitialDuration.set(0.5 * swingDuration);
      swingEndDuration.set(0.5 * swingDuration);
      nextTransferInitialDuration.set(0.5 * transferDuration);
      nextTransferEndDuration.set(0.5 * transferDuration);

      yoNextFootstepPlan = new YoFramePose("nextFootstepPlan", worldFrame, registry);
      yoNextNextFootstepPlan = new YoFramePose("nextNextFootstepPlan", worldFrame, registry);
      yoNextNextNextFootstepPlan = new YoFramePose("nextNextNextFootstepPlan", worldFrame, registry);

      yoNextFootstepPose = new YoFramePose("nextFootstepPose", worldFrame, registry);
      yoNextNextFootstepPose = new YoFramePose("nextNextFootstepPose", worldFrame, registry);
      yoNextNextNextFootstepPose = new YoFramePose("nextNextNextFootstepPose", worldFrame, registry);

      yoNextFootstepPolygon = new YoFrameConvexPolygon2d("nextFootstep", "", worldFrame, 4, registry);
      yoNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextFootstep", "", worldFrame, 4, registry);
      yoNextNextNextFootstepPolygon = new YoFrameConvexPolygon2d("nextNextNextFootstep", "", worldFrame, 4, registry);

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));

      YoGraphicShape nextFootstepViz = new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0);
      YoGraphicShape nextNextFootstepViz = new YoGraphicShape("nextNextFootstep", footstepGraphics, yoNextNextFootstepPose, 1.0);
      YoGraphicShape nextNextNextFootstepViz = new YoGraphicShape("nextNextNextFootstep", footstepGraphics, yoNextNextNextFootstepPose, 1.0);

      YoArtifactPolygon nextFootstepArtifact =  new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextFootstepArtifact =  new YoArtifactPolygon("nextNextFootstep", yoNextNextFootstepPolygon, Color.blue, false);
      YoArtifactPolygon nextNextNextFootstepArtifact =  new YoArtifactPolygon("nextNextNextFootstep", yoNextNextNextFootstepPolygon, Color.blue, false);

      yoGraphicsListRegistry.registerYoGraphic("dummy", nextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic("dummy", nextNextFootstepViz);
      yoGraphicsListRegistry.registerYoGraphic("dummy", nextNextNextFootstepViz);

      yoGraphicsListRegistry.registerArtifact("dummy", nextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact("dummy", nextNextFootstepArtifact);
      yoGraphicsListRegistry.registerArtifact("dummy", nextNextNextFootstepArtifact);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.setShowOnStart(true);
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      Thread myThread = new Thread(scs);
      myThread.start();
   }

   private void setupFeetFrames(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(stepWidth / 2.0));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         currentFootPoses.put(robotSide, new YoFramePose(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
         soleFrames.put(robotSide, contactableFoot.getSoleFrame());
      }

      midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, yoGraphicsListRegistry);

      footstepTestHelper = new FootstepTestHelper(contactableFeet);
   }

   private final FramePose3D footstepPose = new FramePose3D();
   private double initialTime;
   private final FootstepTiming firstTiming = new FootstepTiming();
   private final FootstepTiming secondTiming = new FootstepTiming();
   private final FootstepTiming thirdTiming = new FootstepTiming();
   private void initialize()
   {
      int index = 0;
      for (Footstep footstep : footstepTestHelper.createFootsteps(stepWidth, stepLength, numberOfFootstepsToCreate))
      {
         footstep.getPose(footstepPose);
         plannedFootsteps.get(index).setPose(footstepPose);

         index++;
      }

      for (RobotSide robotSide : RobotSide.values)
         contactStates.get(robotSide).setFullyConstrained();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      icpPlanner.clearPlan();
      firstTiming.setTimings(swingDuration, initialTransferDuration);
      secondTiming.setTimings(swingDuration, transferDuration);
      thirdTiming.setTimings(swingDuration, transferDuration);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(0), firstTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(1), secondTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(2), thirdTiming);

      RobotSide supportSide = plannedFootsteps.get(0).getRobotSide().getOppositeSide();

      icpPlanner.setSupportLeg(supportSide);
      icpPlanner.initializeForTransfer(yoTime.getDoubleValue());
      icpPlanner.compute(yoTime.getDoubleValue());

      initialTime = yoTime.getDoubleValue();
      updateViz();
   }

   private boolean firstTick = true;

   public void updateGraphic()
   {
      if (firstTick)
      {
         initialize();
         firstTick = false;
      }


      double initialTransferDuration = currentTransferInitialDuration.getDoubleValue() + currentTransferEndDuration.getDoubleValue();
      double initialSwingDuration = swingInitialDuration.getDoubleValue() + swingEndDuration.getDoubleValue();
      double nextTransferDuration = nextTransferInitialDuration.getDoubleValue() + nextTransferEndDuration.getDoubleValue();

      double initialTransferAlpha = currentTransferInitialDuration.getDoubleValue() / initialTransferDuration;
      double initialSwingAlpha = swingInitialDuration.getDoubleValue() / initialSwingDuration;
      double nextTransferAlpha = nextTransferInitialDuration.getDoubleValue() / nextTransferDuration;


      icpPlanner.clearPlan();
      firstTiming.setTimings(initialSwingDuration, initialTransferDuration);
      secondTiming.setTimings(swingDuration, nextTransferDuration);
      thirdTiming.setTimings(swingDuration, transferDuration);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(0), firstTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(1), secondTiming);
      icpPlanner.addFootstepToPlan(plannedFootsteps.get(2), thirdTiming);
      icpPlanner.setTransferDurationAlpha(0, initialTransferAlpha);
      icpPlanner.setTransferDurationAlpha(1, nextTransferAlpha);
      icpPlanner.setSwingDurationAlpha(0, initialSwingAlpha);

      icpPlanner.initializeForTransfer(initialTime);
      icpPlanner.compute(initialTime);

      updateViz();
   }


   private final FramePose3D nextPlannedFootstep = new FramePose3D();
   private final FramePose3D nextNextPlannedFootstep = new FramePose3D();
   private final FramePose3D nextNextNextPlannedFootstep = new FramePose3D();

   private final FrameConvexPolygon2d footstepPolygon = new FrameConvexPolygon2d();
   private final FrameConvexPolygon2d tempFootstepPolygonForShrinking = new FrameConvexPolygon2d();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();

   private void updateViz()
   {
      nextPlannedFootstep.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      nextNextPlannedFootstep.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      nextNextNextPlannedFootstep.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      yoNextFootstepPlan.setAndMatchFrame(nextPlannedFootstep);
      yoNextNextFootstepPlan.setAndMatchFrame(nextNextPlannedFootstep);
      yoNextNextNextFootstepPlan.setAndMatchFrame(nextNextNextPlannedFootstep);

      if (plannedFootsteps.get(0) == null)
      {
         yoNextFootstepPose.setToNaN();
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextFootstepPolygon.hide();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (plannedFootsteps.get(0).getPredictedContactPoints() == null)
         plannedFootsteps.get(0).setPredictedContactPoints(contactableFeet.get(plannedFootsteps.get(0).getRobotSide()).getContactPoints2d());

      double polygonShrinkAmount = 0.005;

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(plannedFootsteps.get(0).getSoleReferenceFrame(), plannedFootsteps.get(0).getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose3D nextFootstepPose = new FramePose3D(plannedFootsteps.get(0).getSoleReferenceFrame());
      yoNextFootstepPose.setAndMatchFrame(nextFootstepPose);

      if (plannedFootsteps.get(1) == null)
      {
         yoNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextFootstepPolygon.hide();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (plannedFootsteps.get(1).getPredictedContactPoints() == null)
         plannedFootsteps.get(1).setPredictedContactPoints(contactableFeet.get(plannedFootsteps.get(1).getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(plannedFootsteps.get(1).getSoleReferenceFrame(), plannedFootsteps.get(1).getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose3D nextNextFootstepPose = new FramePose3D(plannedFootsteps.get(1).getSoleReferenceFrame());
      yoNextNextFootstepPose.setAndMatchFrame(nextNextFootstepPose);

      if (plannedFootsteps.get(2) == null)
      {
         yoNextNextNextFootstepPose.setToNaN();
         yoNextNextNextFootstepPolygon.hide();
         return;
      }

      if (plannedFootsteps.get(2).getPredictedContactPoints() == null)
         plannedFootsteps.get(2).setPredictedContactPoints(contactableFeet.get(plannedFootsteps.get(2).getRobotSide()).getContactPoints2d());

      tempFootstepPolygonForShrinking.setIncludingFrameAndUpdate(plannedFootsteps.get(2).getSoleReferenceFrame(), plannedFootsteps.get(2).getPredictedContactPoints());
      convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

      footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
      yoNextNextNextFootstepPolygon.setFrameConvexPolygon2d(footstepPolygon);

      FramePose3D nextNextNextFootstepPose = new FramePose3D(plannedFootsteps.get(2).getSoleReferenceFrame());
      yoNextNextNextFootstepPose.setAndMatchFrame(nextNextNextFootstepPose);

      RobotSide supportSide = plannedFootsteps.get(0).getRobotSide().getOppositeSide();
      FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
      FramePose3D nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
      nextSupportPose.setToZero(plannedFootsteps.get(0).getSoleReferenceFrame());
      nextSupportPose.changeFrame(ReferenceFrame.getWorldFrame());
      footSpoof.setSoleFrame(nextSupportPose);
   }

   public static void main(String[] args)
   {
      ICPPlanTimingVisualizer stepAdjustmentExampleGraphic = new ICPPlanTimingVisualizer();
   }

   private class DummyRobot extends  Robot
   {
      public DummyRobot()
      {
         super("dummyRobot");
      }

      @Override
      public void update()
      {
         super.update();
         updateGraphic();
      }

   }

   private ICPPlannerParameters createICPPlannerParameters()
   {
      return new ContinuousCMPICPPlannerParameters()
      {
         @Override
         public int getNumberOfCoPWayPointsPerFoot()
         {
            return 2;
         }

         /**{@inheritDoc} */
         @Override
         public CoPPointName getExitCoPName()
         {
            return exitCoPName;
         }

         /**{@inheritDoc} */
         @Override
         public CoPPointName getEntryCoPName()
         {
            return entryCoPName;
         }

         /** {@inheritDoc} */
         @Override
         public EnumMap<CoPPointName, Vector2D> getCoPOffsetsInFootFrame()
         {
            Vector2D entryOffset = new Vector2D(0.0, -0.005);
            Vector2D exitOffset = new Vector2D(0.0, 0.025);

            EnumMap<CoPPointName, Vector2D> copOffsets = new EnumMap<>(CoPPointName.class);
            copOffsets.put(entryCoPName, entryOffset);
            copOffsets.put(exitCoPName, exitOffset);

            return copOffsets;
         }

         /** {@inheritDoc} */
         @Override
         public EnumMap<CoPPointName, Vector2D> getCoPForwardOffsetBoundsInFoot()
         {
            Vector2D entryBounds = new Vector2D(0.0, 0.03);
            Vector2D exitBounds = new Vector2D(-0.04, 0.08);

            EnumMap<CoPPointName, Vector2D> copForwardOffsetBounds = new EnumMap<>(CoPPointName.class);
            copForwardOffsetBounds.put(entryCoPName, entryBounds);
            copForwardOffsetBounds.put(exitCoPName, exitBounds);

            return copForwardOffsetBounds;
         }
      };
   }
}
