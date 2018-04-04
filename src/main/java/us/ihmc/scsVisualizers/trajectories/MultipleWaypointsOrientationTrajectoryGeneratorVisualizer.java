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
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MultipleWaypointsOrientationTrajectoryGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.0001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final Random random = new Random(54651L);
   private final ReferenceFrame aFrame = EuclidFrameRandomTools.nextReferenceFrame("aFrame", random, worldFrame);

   private final SimpleOrientationTrajectoryGenerator simpleTraj;
   private final MultipleWaypointsOrientationTrajectoryGenerator testedTraj;

   private final YoFramePoint timeVizAsPosition = new YoFramePoint("timeVizAsPosition", worldFrame, registry);
   private final YoFrameOrientation simpleTrajOrientationViz = new YoFrameOrientation("simpleTrajOrientationViz", worldFrame, registry);
   private final YoFrameVector simpleTrajVelocityDirectionViz = new YoFrameVector("simpleTrajVelocityDirectionViz", worldFrame, registry);

   private final YoFrameOrientation testedTrajOrientationViz = new YoFrameOrientation("testedTrajOrientationViz", worldFrame, registry);
   private final YoFrameVector testedTrajVelocityDirectionViz = new YoFrameVector("testedTrajVelocityDirectionViz", worldFrame, registry);
   private final YoFramePose[] waypointOrientationViz;

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

      testedTraj = new MultipleWaypointsOrientationTrajectoryGenerator("testedTraj", 200, true, worldFrame, registry);
      testedTraj.clear();
      
      int numberOfWaypoints = 5;

      waypointOrientationViz = new YoFramePose[numberOfWaypoints];
      
      FrameQuaternion waypointOrientation = new FrameQuaternion();
      FrameVector3D waypointAngularVelocity = new FrameVector3D();
      
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = numberOfWaypoints == 1 ? trajectoryTime / 2.0 : i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         simpleTraj.getOrientation(waypointOrientation);
         simpleTraj.getAngularVelocity(waypointAngularVelocity);
         testedTraj.appendWaypoint(timeAtWaypoint, waypointOrientation, waypointAngularVelocity);
         
         waypointOrientationViz[i] = new YoFramePose("waypointOrientationViz" + i, worldFrame, registry);
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
      scs.addYoVariableRegistry(registry);
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
         simpleTraj.getOrientation(tempOrientation);
         simpleTrajOrientationViz.set(tempOrientation);
         
         simpleTraj.getAngularVelocity(tempVelocity);
         tempVelocity.normalize();
         simpleTrajVelocityDirectionViz.set(tempVelocity);

         
         testedTraj.getAngularVelocity(tempVelocity);
         tempVelocity.normalize();

         testedTrajVelocityDirectionViz.set(tempVelocity);
         testedTraj.getOrientation(tempOrientation);
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
