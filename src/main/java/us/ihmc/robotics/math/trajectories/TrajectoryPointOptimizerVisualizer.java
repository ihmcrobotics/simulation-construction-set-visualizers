package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;

public class TrajectoryPointOptimizerVisualizer
{
   private enum TestType {
      STRAIGHT,
      SWING,
      LINE,
      SPHERE
   }
   private static final TestType type = TestType.SWING;
   private static final int points = 10;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();

   private final PositionOptimizedTrajectoryGenerator trajectory = new PositionOptimizedTrajectoryGenerator("", registry, graphicsListRegistry, -1, points);

   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalVelocity = new FrameVector3D();
   private final ArrayList<FramePoint3D> waypointPositions = new ArrayList<>();

   public TrajectoryPointOptimizerVisualizer()
   {
      switch (type)
      {
      case SWING:
         setupSwing();
         break;
      case LINE:
         setupLine();
         break;
      case SPHERE:
         setupSphere();
         break;
      default:
         initialPosition.setIncludingFrame(worldFrame, -0.5, -0.5, 0.0);
         initialVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);
         finalPosition.setIncludingFrame(worldFrame, 1.0, 0.0, 1.0);
         finalVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);
         break;
      }

      trajectory.setEndpointConditions(initialPosition, initialVelocity, finalPosition, finalVelocity);
      trajectory.setWaypoints(waypointPositions);
      createGUIAndSleep();
   }

   private void setupSphere()
   {
      Point3D sphereOrigin = new Point3D(0.0, 0.0, 0.7);
      double sphereRadius = 0.5;
      Point3D[] waypointPositions = SpiralBasedAlgorithm.generatePointsOnSphere(sphereOrigin, sphereRadius, points+2);

      initialPosition.setIncludingFrame(worldFrame, waypointPositions[0]);
      initialVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);
      finalPosition.setIncludingFrame(worldFrame, waypointPositions[points-1]);
      finalVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);

      for (int i = 1; i < points-1; i++)
         this.waypointPositions.add(new FramePoint3D(worldFrame, waypointPositions[i]));
   }

   private void setupLine()
   {
      initialPosition.setIncludingFrame(worldFrame, -0.5, 0.0, 0.5);
      initialVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);
      finalPosition.setIncludingFrame(worldFrame, 0.5, 0.0, 0.5);
      finalVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);

      for (int i = 0; i < points; i++)
      {
         double percent = (double)(i+1) / (double)(points+1);
         FramePoint3D waypoint = new FramePoint3D();
         waypoint.interpolate(initialPosition, finalPosition, percent);
         waypointPositions.add(waypoint);
      }
   }

   private void setupSwing()
   {
      initialPosition.setIncludingFrame(worldFrame, -0.5, 0.0, 0.0);
      initialVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);
      finalPosition.setIncludingFrame(worldFrame, 0.5, 0.0, 0.0);
      finalVelocity.setIncludingFrame(worldFrame, 0.0, 0.0, 0.0);
      waypointPositions.add(new FramePoint3D(worldFrame, -0.01, 0.0, 1.0));
      waypointPositions.add(new FramePoint3D(worldFrame, 0.01, 0.0, 1.0));
   }

   private void createGUIAndSleep()
   {
      for (int i = 0; i < waypointPositions.size(); i++)
      {
         YoFramePoint3D yoWaypoint = new YoFramePoint3D("waypoint" + i, worldFrame, registry);
         yoWaypoint.set(waypointPositions.get(i));
         graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("waypoint" + i + "Viz", yoWaypoint, 0.02, YoAppearance.White()));
      }

      YoFramePoint3D yoStart = new YoFramePoint3D("start", worldFrame, registry);
      yoStart.set(initialPosition);
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("start", yoStart, 0.02, YoAppearance.White()));

      YoFramePoint3D yoEnd = new YoFramePoint3D("end", worldFrame, registry);
      yoEnd.set(finalPosition);
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("end", yoEnd, 0.02, YoAppearance.White()));

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(100);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(1.0, 1);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.addYoGraphicsListRegistry(graphicsListRegistry, true);

      trajectory.initialize();
      double time = 0.0;

      while (!trajectory.hasConverged())
      {
         trajectory.compute(0.0);
         scs.setTime(time);
         scs.tickAndUpdate();
         time += 1.0;
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new TrajectoryPointOptimizerVisualizer();
   }
}
