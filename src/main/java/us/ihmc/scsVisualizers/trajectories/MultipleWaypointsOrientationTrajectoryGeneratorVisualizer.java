package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MultipleWaypointsOrientationTrajectoryGeneratorVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.0001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Random random = new Random(54651L);
   private final ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

   private final SimpleOrientationTrajectoryGenerator simpleTraj;
   private final MultipleWaypointsOrientationTrajectoryGenerator testedTraj;

   private final YoFramePoint3D timeVizAsPosition = new YoFramePoint3D("timeVizAsPosition", worldFrame, registry);
   private final YoFrameYawPitchRoll simpleTrajOrientationViz = new YoFrameYawPitchRoll("simpleTrajOrientationViz", worldFrame, registry);
   private final YoFrameVector3D simpleTrajVelocityDirectionViz = new YoFrameVector3D("simpleTrajVelocityDirectionViz", worldFrame, registry);

   private final YoFrameYawPitchRoll testedTrajOrientationViz = new YoFrameYawPitchRoll("testedTrajOrientationViz", worldFrame, registry);
   private final YoFrameVector3D testedTrajVelocityDirectionViz = new YoFrameVector3D("testedTrajVelocityDirectionViz", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll[] waypointOrientationViz;

   private final Point3D initialPosition = new Point3D(-1.0, 0.0, 0.7);
   private final Point3D finalPosition = new Point3D(1.0, 0.0, 0.7);
   private final Point3D interpolatedPosition = new Point3D();

   public MultipleWaypointsOrientationTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      yoGraphicsListRegistry.registerYoGraphic("simpleTrajViz", new YoGraphicCoordinateSystem("simpleTrajOrientationViz", timeVizAsPosition, simpleTrajOrientationViz, 1.0, YoAppearance.Blue()));
      yoGraphicsListRegistry.registerYoGraphic("simpleTrajViz", new YoGraphicVector("simpleTrajVelocityDirectionViz", timeVizAsPosition, simpleTrajVelocityDirectionViz, 1.0, YoAppearance.DarkBlue()));

      yoGraphicsListRegistry.registerYoGraphic("testedTrajViz", new YoGraphicCoordinateSystem("testedTrajOrientationViz", timeVizAsPosition, testedTrajOrientationViz, 1.0, YoAppearance.Red()));
      yoGraphicsListRegistry.registerYoGraphic("testedTrajViz", new YoGraphicVector("testedTrajVelocityDirectionViz", timeVizAsPosition, testedTrajVelocityDirectionViz, 1.0, YoAppearance.DarkRed()));


      simpleTraj = new SimpleOrientationTrajectoryGenerator("simpleTraj", true, worldFrame, registry);
      simpleTraj.setTrajectoryTime(trajectoryTime);
      simpleTraj.setInitialOrientation(new FrameQuaternion(worldFrame, 1.0, 0.2, -0.5));
      simpleTraj.setFinalOrientation(new FrameQuaternion(worldFrame, -0.3, 0.7, 1.0));
//      simpleTraj.setInitialOrientation(new FrameOrientation(worldFrame, 1.0, 0.0, 0.0));
//      simpleTraj.setFinalOrientation(new FrameOrientation(worldFrame, 0.0, 0.0, 0.0));
      simpleTraj.initialize();

      testedTraj = new MultipleWaypointsOrientationTrajectoryGenerator("testedTraj", 200, worldFrame, registry);
      testedTraj.clear();

      int numberOfWaypoints = 5;

      waypointOrientationViz = new YoFramePoseUsingYawPitchRoll[numberOfWaypoints];

      FrameQuaternion waypointOrientation = new FrameQuaternion();
      FrameVector3D waypointAngularVelocity = new FrameVector3D();

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = numberOfWaypoints == 1 ? trajectoryTime / 2.0 : i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         waypointOrientation.setIncludingFrame(simpleTraj.getOrientation());
         waypointAngularVelocity.setIncludingFrame(simpleTraj.getAngularVelocity());
         testedTraj.appendWaypoint(timeAtWaypoint, waypointOrientation, waypointAngularVelocity);

         waypointOrientationViz[i] = new YoFramePoseUsingYawPitchRoll("waypointOrientationViz" + i, worldFrame, registry);
         waypointOrientationViz[i].setOrientation(waypointOrientation);
         interpolatedPosition.interpolate(initialPosition, finalPosition, timeAtWaypoint / trajectoryTime);
         waypointOrientationViz[i].setPosition(interpolatedPosition);
         yoGraphicsListRegistry.registerYoGraphic("waypointViz", new YoGraphicCoordinateSystem("waypointOrientationViz" + i, waypointOrientationViz[i], 0.5, YoAppearance.Black()));
      }
      testedTraj.initialize();

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      Robot robot = new Robot("dummy");
      YoDouble yoTime = robot.getYoTime();
      SimulationConstructionSet scs = new SimulationConstructionSet(robot, parameters);
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      FrameQuaternion tempOrientation = new FrameQuaternion();
      FrameVector3D tempVelocity = new FrameVector3D();

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         testedTraj.compute(t);
         simpleTraj.compute(t);

         interpolatedPosition.interpolate(initialPosition, finalPosition, t / trajectoryTime);
         timeVizAsPosition.set(interpolatedPosition);
         tempOrientation.setIncludingFrame(simpleTraj.getOrientation());
         simpleTrajOrientationViz.set(tempOrientation);

         tempVelocity.setIncludingFrame(simpleTraj.getAngularVelocity());
         tempVelocity.normalize();
         simpleTrajVelocityDirectionViz.set(tempVelocity);


         tempVelocity.setIncludingFrame(testedTraj.getAngularVelocity());
         tempVelocity.normalize();

         testedTrajVelocityDirectionViz.set(tempVelocity);
         tempOrientation.setIncludingFrame(testedTraj.getOrientation());
         testedTrajOrientationViz.set(tempOrientation);
         yoTime.add(dt);
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new MultipleWaypointsOrientationTrajectoryGeneratorVisualizer();
   }
}
