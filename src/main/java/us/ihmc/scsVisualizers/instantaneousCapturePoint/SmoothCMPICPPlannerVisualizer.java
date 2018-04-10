package us.ihmc.scsVisualizers.instantaneousCapturePoint;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Random;

import com.thoughtworks.xstream.io.StreamException;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSmoothCMPPlannerParameters;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.ContinuousCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.configurations.CoPPointName;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptFileLoader;
import us.ihmc.humanoidBehaviors.behaviors.scripts.engine.ScriptObject;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.MidFootZUpGroundFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SmoothCMPICPPlannerVisualizer
{
   private static final boolean SHOW_OLD_PLANNER = false;
   private static final boolean SHOW_WITHOUT_ANGULAR_MOMENTUM = false;

   private static final double mass = 0.1;
   private final double defaultSwingTime = 1.2; //1;
   private final double defaultTransferTime = 0.60; //0.15; //0.1;

   private final double defaultInitialTransferTime = 1.0; //0.1;
   private final double desiredVelocity = 0.62; //0.5;
   private final double defaultStepWidth = 0.25;
   private static final int BUFFER_SIZE = 16000;

   private final static int numberOfSteps = 5;
   private static final double omega = 3.4;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry("SmoothCMPBasedICPPlannerVisualizer");
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final SimulationConstructionSet scs;

   private final YoDouble omega0 = new YoDouble("omega0", registry);

   private final List<YoFramePose> yoNextFootstepPose = new ArrayList<>();
   private final List<YoFrameConvexPolygon2d> yoNextFootstepPolygon = new ArrayList<>();
   private final ArrayList<Updatable> updatables = new ArrayList<>();
   public static final Color defaultLeftColor = new Color(0.85f, 0.35f, 0.65f, 1.0f);
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final YoDouble yoTime;
   private final double dt = 0.001;

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());
   private SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
   private SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private MidFootZUpGroundFrame midFeetZUpFrame;
   private SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();
   private SideDependentList<YoFramePose> currentFootPoses = new SideDependentList<>();
   private BipedSupportPolygons bipedSupportPolygons;

   private SmoothCMPPlannerParameters planParameters;
   private SmoothCMPBasedICPPlanner icpPlanner;
   private SmoothCMPBasedICPPlanner icpPlannerAMOff;

   private ContinuousCMPBasedICPPlanner heelToeICPPlanner;
   private ContinuousCMPBasedICPPlanner cdsICPPlanner;

   private FootstepTestHelper footstepTestHelper;
   private final AtlasPhysicalProperties atlasPhysicalProperties = new AtlasPhysicalProperties();
   private List<Footstep> footsteps;
   private List<FootstepTiming> footstepTimings;
   private BagOfBalls copTrack, cmpTrack, icpTrack, comTrack, icpInitialTrack, icpFinalTrack, comInitialTrack, comFinalTrack, cmpFinalTrack, swTrack,
         estCoMTrack;
   private BagOfBalls copTrackAMOff, cmpTrackAMOff, icpTrackAMOff, comTrackAMOff;
   private BagOfBalls oldICPTrack, oldCMPTrack, oldCoMTrack;
   private YoGraphicPosition cmpInitialTrack;
   private YoGraphicPosition icpTerminalTrack;
   private BagOfBalls copWaypointViz;
   private BagOfBalls cmpWaypointViz;

   private final int simulatedTicksPerGraphicUpdate = 2;

   private final YoBoolean isStanding = new YoBoolean("isStanding", registry);

   private final YoFramePoint heelToeDesiredICP;
   private final YoFramePoint heelToeDesiredCoM;
   private final YoFramePoint heelToeDesiredCMP;
   private final YoFrameVector heelToeDesiredICPVelocity;
   private final YoFrameVector heelToeDesiredCMPVelocity;
   private final YoFramePoint cdsDesiredICP;
   private final YoFramePoint cdsDesiredCoM;
   private final YoFramePoint cdsDesiredCMP;
   private final YoFrameVector cdsDesiredICPVelocity;
   private final YoFrameVector cdsDesiredCMPVelocity;

   private final YoFramePoint desiredCoP;
   private final YoFramePoint desiredCMP;
   private final YoFramePoint desiredICP;
   private final YoFrameVector desiredICPVelocity;
   private final YoFramePoint predictedCoM, predictedSwingFootLocation;
   private final YoFramePoint desiredCoM;
   private final YoFrameVector desiredCoMVelocity;
   private final YoFrameVector desiredCoMAcceleration;
   private final YoFrameVector desiredCoPVelocity;

   private final YoFramePoint desiredCoPAMOff;
   private final YoFramePoint desiredCMPAMOff;
   private final YoFramePoint desiredICPAMOff;
   private final YoFramePoint desiredCoMAMOff;
   private final YoFrameVector desiredICPVelocityAMOff;
   private final YoFrameVector desiredCoMVelocityAMOff;
   private final YoFrameVector desiredCoMAccelerationAMOff;

   private final YoFrameVector desiredReactionForce;
   private final YoGraphicVector desiredReactionForceGraphic;

   private final YoFrameVector desiredReactionForceAMOff;
   private final YoGraphicVector desiredReactionForceGraphicAMOff;

   private final YoGraphicVector heelToeDesiredReactionForceGraphic;
   private final YoGraphicVector cdsDesiredReactionForceGraphic;

   private final YoFrameVector heelToeDesiredReactionForce;
   private final YoFrameVector cdsDesiredReactionForce;

   private final YoFrameVector desiredReactionForceNewton;
   private final YoGraphicVector desiredReactionForceGraphicNewton;

   private FramePoint3D desiredCMPInitial;
   private FramePoint3D desiredICPTerminal;
   private FramePoint3D desiredICPInitial;
   private FramePoint3D desiredICPFinal;
   private FramePoint3D desiredCoMInitial;
   private FramePoint3D desiredCoMFinal;
   private FramePoint3D desiredCMPFinal;
   private List<? extends FramePoint3DReadOnly> icpDesiredInitialPositions;
   private List<? extends FramePoint3DReadOnly> icpDesiredFinalPositions;
   private List<? extends FramePoint3DReadOnly> comDesiredInitialPositions;
   private List<? extends FramePoint3DReadOnly> comDesiredFinalPositions;
   private List<? extends FramePoint3DReadOnly> cmpDesiredFinalPositions;

   private List<YoGraphicLineSegment> icpLineSegments = new ArrayList<>();

   // FIXME: copied from ICPPlannerVisualizer, does not work for either of them
   private static final boolean USE_SCRIPT = false;
   private static final boolean USE_PITCH_ROLL_OSCILLATIONS_WITH_HIGH_Z = false;
   private static final boolean USE_RANDOM_CONTACT_POINTS = false;
   private final double footZShift = 0.0 * 1.5;
   private final Random random = new Random(21616L);

   private final String namePrefix = "SmoothCMPICPPlannerVisualizer";

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   public SmoothCMPICPPlannerVisualizer()
   {
      setupPlanner();
      isStanding.set(true);

      omega0.set(omega);
      icpPlanner.setOmega0(omega0.getDoubleValue());
      if (SHOW_OLD_PLANNER)
      {
         heelToeICPPlanner.setOmega0(omega0.getDoubleValue());
         cdsICPPlanner.setOmega0(omega0.getDoubleValue());
      }
      if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
      {
         icpPlannerAMOff.setOmega0(omega0.getDoubleValue());
      }

      for (int i = 0; i < planParameters.getNumberOfFootstepsToConsider(); i++)
      {
         YoFrameConvexPolygon2d nextFootPolygon = new YoFrameConvexPolygon2d("nextFootstep" + i, "", worldFrame, 4, registry);
         yoNextFootstepPolygon.add(nextFootPolygon);
         graphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep" + i, nextFootPolygon, Color.blue, false));
      }

      if (SHOW_OLD_PLANNER)
      {
         heelToeDesiredCoM = new YoFramePoint("heelToeDesiredCoM", worldFrame, registry);
         heelToeDesiredICP = new YoFramePoint("heelToeDesiredICP", worldFrame, registry);
         heelToeDesiredCMP = new YoFramePoint("heelToeDesiredCMP", worldFrame, registry);
         heelToeDesiredICPVelocity = new YoFrameVector("heelToeDesiredICPVelocity", worldFrame, registry);
         heelToeDesiredCMPVelocity = new YoFrameVector("heelToeDesiredCMPVelocity", worldFrame, registry);

         cdsDesiredCoM = new YoFramePoint("cdsDesiredCoM", worldFrame, registry);
         cdsDesiredICP = new YoFramePoint("cdsDesiredICP", worldFrame, registry);
         cdsDesiredCMP = new YoFramePoint("cdsDesiredCMP", worldFrame, registry);
         cdsDesiredICPVelocity = new YoFrameVector("cdsDesiredICPVelocity", worldFrame, registry);
         cdsDesiredCMPVelocity = new YoFrameVector("cdsDesiredCMPVelocity", worldFrame, registry);

         heelToeDesiredReactionForce = new YoFrameVector("heelToeDesiredReactionForce", worldFrame, registry);
         heelToeDesiredReactionForceGraphic = new YoGraphicVector("heelToeDesiredReactionForceViz", heelToeDesiredCMP, heelToeDesiredReactionForce,
                                                                  YoAppearance.Orange());
         cdsDesiredReactionForce = new YoFrameVector("cdsToeDesiredReactionForce", worldFrame, registry);
         cdsDesiredReactionForceGraphic = new YoGraphicVector("cdsToeDesiredReactionForceViz", cdsDesiredCMP, cdsDesiredReactionForce,
                                                              YoAppearance.Darkorange());
         graphicsListRegistry.registerYoGraphic("Heel Toe Desired Reaction Force", heelToeDesiredReactionForceGraphic);
         graphicsListRegistry.registerYoGraphic("CDS Desired Reaction Force", cdsDesiredReactionForceGraphic);
      }
      else
      {
         heelToeDesiredCoM = null;
         heelToeDesiredICP = null;
         heelToeDesiredCMP = null;
         heelToeDesiredICPVelocity = null;
         heelToeDesiredCMPVelocity = null;

         cdsDesiredCoM = null;
         cdsDesiredICP = null;
         cdsDesiredCMP = null;
         cdsDesiredICPVelocity = null;
         cdsDesiredCMPVelocity = null;

         heelToeDesiredReactionForce = null;
         heelToeDesiredReactionForceGraphic = null;
         cdsDesiredReactionForce = null;
         cdsDesiredReactionForceGraphic = null;
      }

      if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
      {
         desiredCoPAMOff = new YoFramePoint("desiredCoPAMOff", worldFrame, registry);
         desiredCMPAMOff = new YoFramePoint("desiredCMPAMOff", worldFrame, registry);
         desiredICPAMOff = new YoFramePoint("desiredICPAMOff", worldFrame, registry);
         desiredCoMAMOff = new YoFramePoint("desiredCoMAMOff", worldFrame, registry);
         desiredICPVelocityAMOff = new YoFrameVector("desiredICPVelocityAMOff", worldFrame, registry);
         desiredCoMVelocityAMOff = new YoFrameVector("desiredCoMVelocityAMOff", worldFrame, registry);
         desiredCoMAccelerationAMOff = new YoFrameVector("desiredCoMAccelerationAMOff", worldFrame, registry);

         desiredReactionForceAMOff = new YoFrameVector("desiredReactionForceAMOff", worldFrame, registry);
         desiredReactionForceGraphicAMOff = new YoGraphicVector("desiredReactionForceAMOffViz", desiredCoPAMOff, desiredReactionForceAMOff, YoAppearance.Red());
         graphicsListRegistry.registerYoGraphic("Angular Momentum Off Desired Reaction Force", desiredReactionForceGraphicAMOff);
      }
      else
      {
         desiredCoPAMOff = null;
         desiredCMPAMOff = null;
         desiredICPAMOff = null;
         desiredCoMAMOff = null;
         desiredICPVelocityAMOff = null;
         desiredCoMVelocityAMOff = null;
         desiredCoMAccelerationAMOff = null;

         desiredReactionForceAMOff = null;
         desiredReactionForceGraphicAMOff = null;
      }

      desiredCoP = new YoFramePoint("desiredCoP", worldFrame, registry);
      desiredCMP = new YoFramePoint("desiredCMP", worldFrame, registry);
      desiredICP = new YoFramePoint("desiredICP", worldFrame, registry);
      desiredICPVelocity = new YoFrameVector("desiredICPVelocity", worldFrame, registry);
      predictedCoM = new YoFramePoint("predictedCoM", worldFrame, registry);
      predictedSwingFootLocation = new YoFramePoint("estSwF", worldFrame, registry);
      desiredCoM = new YoFramePoint("desiredCoM", worldFrame, registry);
      desiredCoMVelocity = new YoFrameVector("desiredCoMVelocity", worldFrame, registry);
      desiredCoMAcceleration = new YoFrameVector("desiredCoMAcceleration", worldFrame, registry);
      desiredCMPInitial = new FramePoint3D();
      desiredICPTerminal = new FramePoint3D();
      desiredICPInitial = new FramePoint3D();
      desiredICPFinal = new FramePoint3D();
      desiredCoMInitial = new FramePoint3D();
      desiredCoMFinal = new FramePoint3D();
      desiredCMPFinal = new FramePoint3D();
      desiredCoPVelocity = new YoFrameVector("desiredCoPVelocity", worldFrame, registry);

      desiredReactionForce = new YoFrameVector("desiredReactionForce", worldFrame, registry);
      desiredReactionForceGraphic = new YoGraphicVector("desiredReactionForceViz", desiredCoP, desiredReactionForce, YoAppearance.Blue());
      desiredReactionForceGraphic.setVisible(false);
      //graphicsListRegistry.registerYoGraphic("Desired Reaction Force", desiredReactionForceGraphic);

      desiredReactionForceNewton = new YoFrameVector("desiredReactionForceNewton", worldFrame, registry);
      desiredReactionForceGraphicNewton = new YoGraphicVector("desiredReactionForceNewtonViz", desiredCoP, desiredReactionForceNewton, YoAppearance.Purple());
      //graphicsListRegistry.registerYoGraphic("Desired Reaction Force Newton", desiredReactionForceGraphicNewton);

      icpDesiredInitialPositions = new ArrayList();
      icpDesiredFinalPositions = new ArrayList();
      cmpDesiredFinalPositions = new ArrayList();

      for (int i = 0; i < 50; ++i)
      {
         YoGraphicLineSegment lineSegment = new YoGraphicLineSegment(namePrefix + i, "ICPLineSegment", worldFrame, YoAppearance.Red(), registry);
         graphicsListRegistry.registerYoGraphic("ICPLine" + i, lineSegment);
         icpLineSegments.add(lineSegment);
      }

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<Point2D>();
      for (FramePoint2D point : contactableFeet.get(RobotSide.LEFT).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));
      for (int i = 0; i < planParameters.getNumberOfFootstepsToConsider(); i++)
      {
         YoFramePose nextPose = new YoFramePose("nextFootstepPose" + i, worldFrame, registry);
         yoNextFootstepPose.add(nextPose);
         graphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep" + i, footstepGraphics, nextPose, 1.0));
      }
      if (USE_SCRIPT)
      {
         footsteps = createScriptBasedFootstepProvider("LongSidesteps.xml");
      }
      else
      {
         footsteps = footstepTestHelper.createFootsteps(defaultStepWidth, desiredVelocity * (defaultSwingTime + defaultTransferTime), numberOfSteps);
      }

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

      oldCMPTrack = new BagOfBalls(50, 0.002, "OldeCMP", YoAppearance.PapayaWhip(), registry, graphicsListRegistry);
      oldICPTrack = new BagOfBalls(50, 0.0015, "OldICP", YoAppearance.Orange(), YoGraphicPosition.GraphicType.BALL_WITH_ROTATED_CROSS, registry,
                                   graphicsListRegistry);
      oldCoMTrack = new BagOfBalls(50, 0.0015, "OldCoM", YoAppearance.Black(), registry, graphicsListRegistry);

      copTrackAMOff = new BagOfBalls(50, 0.003, "CoPAMOff", YoAppearance.AliceBlue(), registry, graphicsListRegistry);
      cmpTrackAMOff = new BagOfBalls(50, 0.003, "eCMPAMOff", YoAppearance.Purple(), registry, graphicsListRegistry);
      icpTrackAMOff = new BagOfBalls(50, 0.001, "ICPAMOff", YoAppearance.Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS, registry,
                                     graphicsListRegistry);
      comTrackAMOff = new BagOfBalls(50, 0.001, "CoMAMOff", YoAppearance.Black(), registry, graphicsListRegistry);

      copTrack = new BagOfBalls(50, 0.005, "CoP", YoAppearance.AliceBlue(), registry, graphicsListRegistry);
      cmpTrack = new BagOfBalls(50, 0.005, "eCMP", YoAppearance.Purple(), registry, graphicsListRegistry);
      icpTrack = new BagOfBalls(50, 0.002, "ICP", YoAppearance.Yellow(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS, registry, graphicsListRegistry);
      comTrack = new BagOfBalls(50, 0.002, "CoM", YoAppearance.Black(), registry, graphicsListRegistry);
      icpInitialTrack = new BagOfBalls(50, 0.004, "ICPInitial", YoAppearance.Pink(), registry, graphicsListRegistry);
      icpFinalTrack = new BagOfBalls(50, 0.004, "ICPFinal", YoAppearance.Red(), registry, graphicsListRegistry);
      comInitialTrack = new BagOfBalls(50, 0.004, "CoMInitial", YoAppearance.Blue(), registry, graphicsListRegistry);
      comFinalTrack = new BagOfBalls(50, 0.004, "CoMFinal", YoAppearance.Green(), registry, graphicsListRegistry);
      cmpFinalTrack = new BagOfBalls(17, 0.010, "CMPFinal", YoAppearance.Yellow(), registry, graphicsListRegistry);
      estCoMTrack = new BagOfBalls(50, 0.004, "predictedCoM", YoAppearance.Green(), registry, graphicsListRegistry);
      swTrack = new BagOfBalls(50, 0.004, "swFoot", YoAppearance.Orange(), registry, graphicsListRegistry);
      
      YoArtifactPosition desiredCMPArtifact = new YoArtifactPosition("desiredCMP", desiredCMP.getYoX(), desiredCMP.getYoY(), GraphicType.BALL_WITH_CROSS, Color.RED, 0.01);
      YoArtifactPosition desiredCoPArtifact = new YoArtifactPosition("desiredCoP", desiredCoP.getYoX(), desiredCoP.getYoY(), GraphicType.BALL, Color.BLUE, 0.01);
      graphicsListRegistry.registerArtifact("desiredCMP", desiredCMPArtifact);
      graphicsListRegistry.registerArtifact("desiredCoP", desiredCoPArtifact);

      cmpInitialTrack = new YoGraphicPosition("CMPInitial", "", registry, 0.01, YoAppearance.Blue());
      graphicsListRegistry.registerYoGraphic("CMP", cmpInitialTrack);
      icpTerminalTrack = new YoGraphicPosition("ICPTerminal", "", registry, 0.01, YoAppearance.Indigo());
      graphicsListRegistry.registerYoGraphic("ICP", icpTerminalTrack);

      copWaypointViz = new BagOfBalls(registry, graphicsListRegistry);
      cmpWaypointViz = new BagOfBalls(50, 0.015, "CMP", YoAppearance.Green(), registry, graphicsListRegistry);
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
      PrintTools.debug("Sleeping forever");
   }

   private List<FootstepTiming> getFootstepTimings()
   {
      List<FootstepTiming> timings = new ArrayList<>(footsteps.size());
      timings.add(new FootstepTiming(defaultSwingTime, defaultInitialTransferTime));
      for (int i = 1; i < footsteps.size(); i++)
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

      for (RobotSide robotSide : RobotSide.values)
         contactStates.get(robotSide).setFullyConstrained();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      //FIXME
      //icpPlanner.initializeForStanding(yoTime.getDoubleValue());

      while (currentStepCount < footsteps.size() + 1)
      {
         nextFootsteps.clear();
         nextFootstepTimings.clear();
         int numberOfNextSteps = footsteps.size() - currentStepCount < planParameters.getNumberOfFootstepsToConsider() ? footsteps.size() - currentStepCount
               : planParameters.getNumberOfFootstepsToConsider();
         for (int i = 0; i < numberOfNextSteps; i++)
         {
            nextFootsteps.add(footsteps.get(currentStepCount + i));
            nextFootstepTimings.add(timings.get(currentStepCount + i));
         }
         updateFootStepViz(nextFootsteps, nextFootstepTimings);
         registerFootsteps(nextFootsteps, nextFootstepTimings);

         Footstep nextFootstep = footsteps.get(footsteps.size() - 1);
         if (!(currentStepCount == footsteps.size() && inDoubleSupport.getBooleanValue()))
         {
            nextFootstep = nextFootsteps.get(0);
         }

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
         callUpdatables();

         if (inDoubleSupport.getBooleanValue())
         {
            RobotSide transferToSide = nextFootstep.getRobotSide().getOppositeSide();
            if(currentStepCount == footsteps.size())
            {
               icpPlanner.setTransferToSide(transferToSide.getOppositeSide());
               icpPlanner.initializeForStanding(yoTime.getDoubleValue());
            }
            else
            {
               icpPlanner.setTransferToSide(transferToSide);
               icpPlanner.initializeForTransfer(yoTime.getDoubleValue());
            }

            if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
            {
               if(currentStepCount == footsteps.size())
               {
                  icpPlannerAMOff.setTransferToSide(transferToSide.getOppositeSide());
                  icpPlannerAMOff.initializeForStanding(yoTime.getDoubleValue());
               }
               else
               {
                  icpPlannerAMOff.setTransferToSide(transferToSide);
                  icpPlannerAMOff.initializeForTransfer(yoTime.getDoubleValue());
               }
            }

            if (SHOW_OLD_PLANNER)
            {
               heelToeICPPlanner.setTransferToSide(transferToSide);
               heelToeICPPlanner.initializeForTransfer(yoTime.getDoubleValue());
               cdsICPPlanner.setTransferToSide(transferToSide);
               cdsICPPlanner.initializeForTransfer(yoTime.getDoubleValue());
            }

            inDoubleSupport.set(false);
         }
         else if (nextFootstep != null)
         {
            RobotSide supportSide = nextFootstep.getRobotSide().getOppositeSide();
            icpPlanner.setSupportLeg(supportSide);
            icpPlanner.initializeForSingleSupport(yoTime.getDoubleValue());

            if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
            {
               icpPlannerAMOff.setSupportLeg(supportSide);
               icpPlannerAMOff.initializeForSingleSupport(yoTime.getDoubleValue());
            }

            if (SHOW_OLD_PLANNER)
            {
               heelToeICPPlanner.setSupportLeg(supportSide);
               heelToeICPPlanner.initializeForSingleSupport(yoTime.getDoubleValue());
               cdsICPPlanner.setSupportLeg(supportSide);
               cdsICPPlanner.initializeForSingleSupport(yoTime.getDoubleValue());
            }

            FootSpoof footSpoof = contactableFeet.get(supportSide.getOppositeSide());
            FramePose3D nextSupportPose = footPosesAtTouchdown.get(supportSide.getOppositeSide());
            nextSupportPose.setToZero(nextFootstep.getSoleReferenceFrame());
            nextSupportPose.changeFrame(worldFrame);
            footSpoof.setSoleFrame(nextSupportPose);
            if (nextFootstep.getPredictedContactPoints() == null)
               contactStates.get(nextFootstep.getRobotSide()).setContactFramePoints(footSpoof.getContactPoints2d());
            else
               contactStates.get(nextFootstep.getRobotSide()).setContactPoints(nextFootstep.getPredictedContactPoints());

            inDoubleSupport.set(true);
         }

         List<CoPPointsInFoot> copList = icpPlanner.getCoPWaypoints();
         copWaypointViz.reset();
         for (int i = 0; i < copList.size(); i++)
         {
            CoPPointsInFoot copPoints = copList.get(i);
            for (int j = 0; j < copPoints.getCoPPointList().size(); j++)
            {
               FramePoint3D tempPoint = new FramePoint3D(copPoints.getWaypointInWorldFrameReadOnly(j));
               tempPoint.add(0.0, 0.0, 0.05);
               copWaypointViz.setBall(tempPoint);
            }
         }

         double totalTime = 1.0;
         if (!(currentStepCount == footsteps.size() && !inDoubleSupport.getBooleanValue()))
         {
            totalTime = inDoubleSupport.getBooleanValue() ? nextFootstepTimings.get(0).getSwingTime() : nextFootstepTimings.get(0).getTransferTime();
         }
         for (double time = 0; time < totalTime; time += dt)
         {
            simulateOneTick();
         }

         icpPlanner.updateListeners();

         if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
         {
            icpPlannerAMOff.updateListeners();
         }

         for (RobotSide robotSide : RobotSide.values)
            contactStates.get(robotSide).setFullyConstrained();

         if (currentStepCount == footsteps.size() && !inDoubleSupport.getBooleanValue())
         {
            break;
         }

         currentStepCount = inDoubleSupport.getBooleanValue() ? currentStepCount + 1 : currentStepCount;
      }
   }

   private int counter = 0;

   private void simulateOneTick()
   {
      callUpdatables();
      updateFootViz();
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      icpPlanner.compute(yoTime.getDoubleValue());

      icpPlanner.getDesiredCenterOfPressurePosition(desiredCoP);
      icpPlanner.getDesiredCentroidalMomentumPivotPosition(desiredCMP);
      icpPlanner.getDesiredCapturePointPosition(desiredICP);
      icpPlanner.getDesiredCapturePointVelocity(desiredICPVelocity);
      icpPlanner.getPredictedCenterOfMassPosition(predictedCoM, yoTime.getDoubleValue());
      icpPlanner.getPredictedSwingFootPosition(predictedSwingFootLocation, yoTime.getDoubleValue());

      icpPlanner.getDesiredCenterOfMassPosition(desiredCoM);
      icpPlanner.getDesiredCenterOfMassVelocity(desiredCoMVelocity);
      icpPlanner.getDesiredCenterOfMassAcceleration(desiredCoMAcceleration);

      computeReactionForceFromCMPPosition(desiredCoM, desiredCMP, desiredReactionForce);
      computeReactionForceFromNewton(desiredCoMAcceleration, desiredReactionForceNewton);

      if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
      {
         icpPlannerAMOff.compute(yoTime.getDoubleValue());

         icpPlannerAMOff.getDesiredCenterOfPressurePosition(desiredCoPAMOff);
         icpPlannerAMOff.getDesiredCentroidalMomentumPivotPosition(desiredCMPAMOff);
         icpPlannerAMOff.getDesiredCapturePointPosition(desiredICPAMOff);
         icpPlannerAMOff.getDesiredCapturePointVelocity(desiredICPVelocityAMOff);

         icpPlannerAMOff.getDesiredCenterOfMassPosition(desiredCoMAMOff);
         icpPlannerAMOff.getDesiredCenterOfMassVelocity(desiredCoMVelocityAMOff);
         icpPlannerAMOff.getDesiredCenterOfMassAcceleration(desiredCoMAccelerationAMOff);

         computeReactionForceFromCMPPosition(desiredCoMAMOff, desiredCMPAMOff, desiredReactionForceAMOff);
      }

      if (SHOW_OLD_PLANNER)
      {
         heelToeICPPlanner.compute(yoTime.getDoubleValue());
         heelToeICPPlanner.getDesiredCapturePointPosition(heelToeDesiredICP);
         heelToeICPPlanner.getDesiredCenterOfMassPosition(heelToeDesiredCoM);
         heelToeICPPlanner.getDesiredCentroidalMomentumPivotPosition(heelToeDesiredCMP);

         heelToeICPPlanner.getDesiredCapturePointVelocity(heelToeDesiredICPVelocity);
         heelToeICPPlanner.getDesiredCentroidalMomentumPivotVelocity(heelToeDesiredCMPVelocity);

         computeReactionForceFromCMPPosition(heelToeDesiredCoM, heelToeDesiredCMP, heelToeDesiredReactionForce);

         cdsICPPlanner.compute(yoTime.getDoubleValue());
         cdsICPPlanner.getDesiredCapturePointPosition(cdsDesiredICP);
         cdsICPPlanner.getDesiredCenterOfMassPosition(cdsDesiredCoM);
         cdsICPPlanner.getDesiredCentroidalMomentumPivotPosition(cdsDesiredCMP);

         cdsICPPlanner.getDesiredCapturePointVelocity(cdsDesiredICPVelocity);
         cdsICPPlanner.getDesiredCentroidalMomentumPivotVelocity(cdsDesiredCMPVelocity);

         computeReactionForceFromCMPPosition(cdsDesiredCoM, cdsDesiredCMP, cdsDesiredReactionForce);
      }

      icpDesiredInitialPositions = icpPlanner.getInitialDesiredCapturePointPositions();
      icpDesiredFinalPositions = icpPlanner.getFinalDesiredCapturePointPositions();
      comDesiredInitialPositions = icpPlanner.getInitialDesiredCenterOfMassPositions();
      comDesiredFinalPositions = icpPlanner.getFinalDesiredCenterOfMassPositions();
      //      cmpDesiredFinalPositions = icpGenerator.getCMPPositionDesiredList();

      if (counter++ % simulatedTicksPerGraphicUpdate == 0)
      {
         FramePoint3D desiredCoP = new FramePoint3D(this.desiredCoP);
         desiredCoP.setZ(0.10);
         copTrack.setBallLoop(desiredCoP);

         FramePoint3D desiredCMP = new FramePoint3D(this.desiredCMP);
         desiredCMP.setZ(0.10);
         cmpTrack.setBallLoop(desiredCMP);

         FramePoint3D desiredICP = new FramePoint3D(this.desiredICP);
         desiredICP.setZ(0.10);
         icpTrack.setBallLoop(desiredICP);

         FramePoint3D desiredCoM = new FramePoint3D(this.desiredCoM);
         desiredCoM.setZ(0.15);
         comTrack.setBallLoop(desiredCoM);

         if (SHOW_OLD_PLANNER)
         {
            FramePoint3D oldDesiredCMP = new FramePoint3D(this.heelToeDesiredCMP);
            oldDesiredCMP.setZ(0.05);
            oldCMPTrack.setBallLoop(oldDesiredCMP);

            FramePoint3D oldDesiredICP = new FramePoint3D(this.heelToeDesiredICP);
            oldDesiredICP.setZ(0.05);
            oldICPTrack.setBallLoop(oldDesiredICP);

            FramePoint3D oldDesiredCoM = new FramePoint3D(this.heelToeDesiredCoM);
            oldDesiredCoM.setZ(0.075);
            oldCoMTrack.setBallLoop(oldDesiredCoM);
         }

         if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
         {
            FramePoint3D desiredCoPAMOff = new FramePoint3D(this.desiredCoPAMOff);
            desiredCoPAMOff.setZ(0.08);
            copTrackAMOff.setBallLoop(desiredCoPAMOff);

            FramePoint3D desiredCMPAMOff = new FramePoint3D(this.desiredCMPAMOff);
            desiredCMPAMOff.setZ(0.08);
            cmpTrackAMOff.setBallLoop(desiredCMPAMOff);

            FramePoint3D desiredICPAMOff = new FramePoint3D(this.desiredICPAMOff);
            desiredICPAMOff.setZ(0.08);
            icpTrackAMOff.setBallLoop(desiredICPAMOff);

            FramePoint3D desiredCoMAMOff = new FramePoint3D(this.desiredCoMAMOff);
            desiredCoMAMOff.setZ(0.13);
            comTrackAMOff.setBallLoop(desiredCoMAMOff);
         }

         FramePoint3D predictedCoM = new FramePoint3D(this.predictedCoM);
         predictedCoM.setZ(0.20);
         estCoMTrack.setBallLoop(predictedCoM);

         FramePoint3D predictedSwingFootLocation = new FramePoint3D(this.predictedSwingFootLocation);
         predictedSwingFootLocation.add(0, 0, 0.05);
         //swTrack.setBallLoop(predictedSwingFootLocation);

         icpInitialTrack.reset();
         icpFinalTrack.reset();
         comInitialTrack.reset();
         comFinalTrack.reset();

         for (int i = 0; i < icpPlanner.getTotalNumberOfSegments(); i++)
         {
            desiredICPInitial.set(icpDesiredInitialPositions.get(i));
            desiredICPInitial.setZ(0.10);
            icpInitialTrack.setBallLoop(desiredICPInitial);

            desiredICPFinal.set(icpDesiredFinalPositions.get(i));
            desiredICPFinal.setZ(0.10);
            icpFinalTrack.setBallLoop(desiredICPFinal);

            desiredCoMInitial.set(comDesiredInitialPositions.get(i));
            desiredCoMInitial.setZ(0.15);
            comInitialTrack.setBallLoop(desiredCoMInitial);

            desiredCoMFinal.set(comDesiredFinalPositions.get(i));
            desiredCoMFinal.setZ(0.15);
            comFinalTrack.setBallLoop(desiredCoMFinal);
         }
      }
      yoTime.add(dt);
      scs.tickAndUpdate();
   }

   private void callUpdatables()
   {
      for (Updatable updatable : updatables)
         updatable.update(yoTime.getDoubleValue());
   }

   private void registerFootsteps(List<Footstep> footsteps, List<FootstepTiming> footstepTimings)
   {
      icpPlanner.clearPlan();
      if (SHOW_OLD_PLANNER)
      {
         heelToeICPPlanner.clearPlan();
         cdsICPPlanner.clearPlan();
      }
      if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
      {
         icpPlannerAMOff.clearPlan();
      }

      for (int i = 0; i < footsteps.size(); i++)
      {
         icpPlanner.addFootstepToPlan(footsteps.get(i), footstepTimings.get(i));
         if (SHOW_OLD_PLANNER)
         {
            heelToeICPPlanner.addFootstepToPlan(footsteps.get(i), footstepTimings.get(i));
            cdsICPPlanner.addFootstepToPlan(footsteps.get(i), footstepTimings.get(i));
         }
         if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
         {
            icpPlannerAMOff.addFootstepToPlan(footsteps.get(i), footstepTimings.get(i));
         }
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
         ReferenceFrame footFrame = ReferenceFrame.constructFrameWithUnchangingTranslationFromParent("FootstepFrame", worldFrame,
                                                                                                     footstep.getFootstepPose().getPosition());
         tempFootPolygon.setIncludingFrame(footFrame, Vertex2DSupplier.asVertex2DSupplier(footstep.getPredictedContactPoints()));
         tempFootPolygon.changeFrameAndProjectToXYPlane(worldFrame);
         yoNextFootstepPolygon.get(i).set(tempFootPolygon);
         yoNextFootstepPose.get(i).setAndMatchFrame(new FramePose3D(footFrame));
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

   private List<Footstep> createScriptBasedFootstepProvider(String... scriptFileNames)
   {
      ArrayList<Footstep> footsteps = new ArrayList<>();

      for (String scriptFileName : scriptFileNames)
      {
         ScriptFileLoader scriptFileLoader = null;
         try
         {
            scriptFileLoader = new ScriptFileLoader(getClass().getResourceAsStream(scriptFileName));
         }
         catch (IOException | StreamException e)
         {
            System.out.println("Script being loaded: " + scriptFileName);
            e.printStackTrace();
         }
         ArrayList<ScriptObject> scriptObjects = scriptFileLoader.readIntoList();
         scriptFileLoader.close();
         ArrayList<FootstepDataListMessage> footstepDataLists = new ArrayList<>();
         for (ScriptObject scriptObject : scriptObjects)
         {
            if (scriptObject.getScriptObject() instanceof FootstepDataListMessage)
               footstepDataLists.add((FootstepDataListMessage) scriptObject.getScriptObject());
         }

         for (FootstepDataListMessage footstepDataList : footstepDataLists)
         {
            for (FootstepDataMessage footstepData : footstepDataList.getFootstepDataList())
            {
               RobotSide robotSide = RobotSide.fromByte(footstepData.getRobotSide());
               Point3D position = footstepData.getLocation();
               Quaternion orientation = footstepData.getOrientation();
               footsteps.add(footstepTestHelper.createFootstep(robotSide, position, orientation));
            }
         }
      }

      return corruptFootsteps(footsteps);
   }

   private List<Footstep> corruptFootsteps(final List<Footstep> footsteps)
   {
      if (USE_PITCH_ROLL_OSCILLATIONS_WITH_HIGH_Z)
      {
         for (Footstep footstep : footsteps)
            footstep.setZ(footstep.getZ() + footZShift);
      }
      //      else
      //         for (Footstep footstep : footsteps)
      //            footstep.setZ(footstep.getZ() + 0.10);

      if (USE_RANDOM_CONTACT_POINTS)
         for (Footstep footstep : footsteps)
            generateRandomPredictedContactPoints(footstep);

      return footsteps;
   }

   private void generateRandomPredictedContactPoints(Footstep footstep)
   {
      FrameConvexPolygon2D randomSupportPolygon = new FrameConvexPolygon2D(FrameVertex2DSupplier.asFrameVertex2DSupplier(contactableFeet.get(footstep.getRobotSide()).getContactPoints2d()));
      List<FramePoint2D> randomPredictedContactPointList = new ArrayList<>();

      double minX = 0.5 * randomSupportPolygon.getMinX();
      double minY = 0.5 * randomSupportPolygon.getMinY();
      double maxX = 0.5 * randomSupportPolygon.getMaxX();
      double maxY = 0.5 * randomSupportPolygon.getMaxY();
      FramePoint2D randomLineOrigin = EuclidFrameRandomTools.nextFramePoint2D(random, randomSupportPolygon.getReferenceFrame(), minX, maxX, minY, maxY);
      FrameVector2D randomLineVector = EuclidFrameRandomTools.nextFrameVector2D(random, randomSupportPolygon.getReferenceFrame());
      FrameLine2D randomLine = new FrameLine2D(randomLineOrigin, randomLineVector);

      convexPolygonTools.cutPolygonWithLine(randomLine, randomSupportPolygon, RobotSide.generateRandomRobotSide(random));
      randomSupportPolygon.update();

      while (randomSupportPolygon.getNumberOfVertices() < 4)
      {
         FramePoint2D duplicate = EuclidFrameRandomTools.nextFramePoint2D(random, randomSupportPolygon.getReferenceFrame(), 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3);
         duplicate.add(new FramePoint2D(randomSupportPolygon.getVertex(0)));
         randomSupportPolygon.addVertex(duplicate);
         randomSupportPolygon.update();
      }

      if (randomSupportPolygon.getNumberOfVertices() != 4)
         System.out.println();

      for (int i = 0; i < randomSupportPolygon.getNumberOfVertices(); i++)
         randomPredictedContactPointList.add(new FramePoint2D(randomSupportPolygon.getVertex(i)));

      footstep.setPredictedContactPoints(randomPredictedContactPointList);
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
         contactPointsInSoleFrame.add(new Point2D(atlasPhysicalProperties.getFootLengthForControl() / 2.0,
                                                  atlasPhysicalProperties.getToeWidthForControl() / 2.0));
         contactPointsInSoleFrame.add(new Point2D(atlasPhysicalProperties.getFootLengthForControl() / 2.0,
                                                  -atlasPhysicalProperties.getToeWidthForControl() / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-atlasPhysicalProperties.getFootLengthForControl() / 2.0,
                                                  -atlasPhysicalProperties.getFootWidthForControl() / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-atlasPhysicalProperties.getFootLengthForControl() / 2.0,
                                                  atlasPhysicalProperties.getFootWidthForControl() / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose3D startingPose = new FramePose3D(worldFrame);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(defaultStepWidth/2.0));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);
         currentFootPoses.put(robotSide, new YoFramePose(sidePrefix + "FootPose", worldFrame, registry));

         Graphics3DObject footGraphics = new Graphics3DObject();
         AppearanceDefinition footColor = robotSide == RobotSide.LEFT ? YoAppearance.Color(defaultLeftColor) : YoAppearance.Color(defaultRightColor);
         footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
         graphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(sidePrefix + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));

         RigidBody foot = contactableFoot.getRigidBody();
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
      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, soleZUpFrames, registry, graphicsListRegistry);
      footstepTestHelper = new FootstepTestHelper(contactableFeet);
      AtlasSmoothCMPPlannerParameters planParametersNoMomentum = new AtlasSmoothCMPPlannerParameters(atlasPhysicalProperties)
      {
         @Override
         public boolean planWithAngularMomentum()
         {
            return false;
         }
      };
      planParameters = new AtlasSmoothCMPPlannerParameters(atlasPhysicalProperties)
      {
         @Override
         public boolean planWithAngularMomentum()
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
         swingDuration.set(defaultSwingTime);
         transferDuration.set(defaultTransferTime);

         YoDouble swingSplitFraction = new YoDouble("swingSplitFraction" + i, registry);
         YoDouble swingDurationShiftFraction = new YoDouble("swingDurationShiftFraction" + i, registry);
         YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
         swingSplitFraction.set(planParameters.getSwingSplitFraction());
         swingDurationShiftFraction.set(planParameters.getSwingDurationShiftFraction());
         transferSplitFraction.set(planParameters.getSwingSplitFraction());
      }
      int i = numberOfFootstepsToConsider.getIntegerValue();
      YoDouble transferDuration = new YoDouble("transferDuration" + i, registry);
      YoDouble transferSplitFraction = new YoDouble("transferSplitFraction" + i, registry);
      transferDuration.set(defaultTransferTime);
      transferSplitFraction.set(planParameters.getSwingSplitFraction());
      String namePrefix = "TestICPPlanner";
      int numberOfPointsPerFoot = planParameters.getNumberOfCoPWayPointsPerFoot();
      icpPlanner = new SmoothCMPBasedICPPlanner(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
                                                                                                                                              .createFullRobotModel(),
                                                bipedSupportPolygons, contactableFeet, numberOfFootstepsToConsider.getIntegerValue(),
                                                registry, graphicsListRegistry, 9.81);
      icpPlanner.initializeParameters(planParameters);
      icpPlanner.setFinalTransferDuration(1.0);

      ContinuousCMPICPPlannerParameters heelToeICPPlannerParameters = new ContinuousCMPICPPlannerParameters()
      {
         @Override
         public int getNumberOfFootstepsToConsider()
         {
            return 4;
         }

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

      ContinuousCMPICPPlannerParameters cdsICPPlannerParameters = new ContinuousCMPICPPlannerParameters()
      {
         @Override
         public int getNumberOfFootstepsToConsider()
         {
            return 4;
         }

         @Override
         public int getNumberOfCoPWayPointsPerFoot()
         {
            return 1;
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

      if (SHOW_OLD_PLANNER)
      {
         heelToeICPPlanner = new ContinuousCMPBasedICPPlanner(bipedSupportPolygons, contactableFeet, numberOfPointsPerFoot, registry, graphicsListRegistry);
         heelToeICPPlanner.initializeParameters(heelToeICPPlannerParameters);
         heelToeICPPlanner.setFinalTransferDuration(1.0);
         YoVariableRegistry dummyRegistry = new YoVariableRegistry("dummy");
         YoGraphicsListRegistry dummyGrahpics = new YoGraphicsListRegistry();
         cdsICPPlanner = new ContinuousCMPBasedICPPlanner(bipedSupportPolygons, contactableFeet, numberOfPointsPerFoot, dummyRegistry, dummyGrahpics);
         cdsICPPlanner.initializeParameters(cdsICPPlannerParameters);
         cdsICPPlanner.setFinalTransferDuration(1.0);
      }

      if (SHOW_WITHOUT_ANGULAR_MOMENTUM)
      {
         YoVariableRegistry dummyRegistry = new YoVariableRegistry("ICPAMOff");
         YoGraphicsListRegistry dummyGrahpics = new YoGraphicsListRegistry();
         icpPlannerAMOff = new SmoothCMPBasedICPPlanner(new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false)
                                                                                                                                                      .createFullRobotModel(),
                                                        bipedSupportPolygons, contactableFeet, numberOfFootstepsToConsider.getIntegerValue(),
                                                        dummyRegistry, dummyGrahpics, 9.81);
         icpPlannerAMOff.initializeParameters(planParametersNoMomentum);
         icpPlannerAMOff.setFinalTransferDuration(1.0);
      }

      YoGraphicsList yoGraphicsList = new YoGraphicsList("graphicsList");
      ArtifactList artifactList = new ArtifactList("artifactList");

      graphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      graphicsListRegistry.registerArtifactList(artifactList);
   }

   private void computeReactionForceFromCMPPosition(YoFramePoint desiredCoMPosition, YoFramePoint desiredCMPPosition, YoFrameVector desiredReactionForceToPack)
   {
      desiredReactionForceToPack.set(desiredCoMPosition);
      desiredReactionForceToPack.sub(desiredCMPPosition);
      desiredReactionForceToPack.scale(Math.pow(omega, 2.0));
      desiredReactionForceToPack.setZ(9.81);
      desiredReactionForceToPack.scale(mass);
   }

   private void computeReactionForceFromNewton(YoFrameVector desiredCoMAcceleration, YoFrameVector desiredReactionForceToPack)
   {
      desiredReactionForceToPack.set(desiredCoMAcceleration);
      desiredReactionForceToPack.setZ(9.81);
      desiredReactionForceToPack.scale(mass);
   }

   public static void main(String[] args)
   {
      new SmoothCMPICPPlannerVisualizer();
   }

}
