package us.ihmc.robotics.math.trajectories;

import java.util.function.Supplier;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;
import us.ihmc.robotics.math.trajectories.generators.EuclideanTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.EuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class EuclideanTrajectoryPointCalculatorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private enum WaypointGenerator
   {
      SPHERE, LINE
   };

   private final WaypointGenerator waypointGenerator = WaypointGenerator.LINE;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 10.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final EuclideanTrajectoryPointCalculator calculator = new EuclideanTrajectoryPointCalculator();
   private final MultipleWaypointsPositionTrajectoryGenerator traj;

   private final YoFramePoint3D currentPositionViz = new YoFramePoint3D("currentPositionViz", worldFrame, registry);
   private final RecyclingArrayList<YoFrameEuclideanTrajectoryPoint> trajectoryPointsViz;
   private final YoDouble[] relativeWaypointTimes;

   public EuclideanTrajectoryPointCalculatorVisualizer()
   {
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      int numberOfTrajectoryPoints = 20;

      Supplier<YoFrameEuclideanTrajectoryPoint> builder = new Supplier<YoFrameEuclideanTrajectoryPoint>()
      {
         private int i = 0;

         @Override
         public YoFrameEuclideanTrajectoryPoint get()
         {
            String indexAsString = Integer.toString(i++);
            YoFrameEuclideanTrajectoryPoint ret = new YoFrameEuclideanTrajectoryPoint("waypointViz", indexAsString, registry);
            yoGraphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("waypointPosition" + indexAsString, ret.getYoX(), ret.getYoY(), ret.getYoZ(),
                                                                                  0.025, YoAppearance.AliceBlue()));
            return ret;
         }
      };
      trajectoryPointsViz = new RecyclingArrayList<>(numberOfTrajectoryPoints, builder);

      relativeWaypointTimes = new YoDouble[numberOfTrajectoryPoints];
      for (int i = 0; i < relativeWaypointTimes.length; i++)
         relativeWaypointTimes[i] = new YoDouble("relativeWaypointTime" + i, registry);

      yoGraphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("currentPositionViz", currentPositionViz, 0.05, YoAppearance.Red()));

      Point3D[] waypointPositions = null;

      switch (waypointGenerator)
      {
      case SPHERE:
         waypointPositions = generateSphereWaypoints(numberOfTrajectoryPoints);
         break;
      case LINE:
         waypointPositions = generateLineWaypoints(numberOfTrajectoryPoints);
         break;
      default:
         throw new RuntimeException("Should not get there: " + waypointGenerator);
      }

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         calculator.appendTrajectoryPoint(waypointPositions[i]);
      }

      calculator.compute(trajectoryTime);

      FrameEuclideanTrajectoryPointList waypoints = calculator.getTrajectoryPoints();

      traj = new MultipleWaypointsPositionTrajectoryGenerator("traj", calculator.getNumberOfTrajectoryPoints(), ReferenceFrame.getWorldFrame(), registry);
      traj.appendWaypoints(waypoints);
      traj.initialize();

      double previousWaypointTime = 0.0;

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         Point3D position3d = new Point3D();
         Vector3D linearVelocity3d = new Vector3D();
         EuclideanTrajectoryPointBasics waypoint = waypoints.getTrajectoryPoint(i);
         waypoint.getPosition(position3d);
         waypoint.getLinearVelocity(linearVelocity3d);
         trajectoryPointsViz.add().set(waypoint.getTime(), position3d, linearVelocity3d);
         relativeWaypointTimes[i].set(waypoint.getTime() - previousWaypointTime);
         previousWaypointTime = waypoint.getTime();
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj.compute(t);

         currentPositionViz.set(traj.getPosition());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   private Point3D[] generateSphereWaypoints(int numberOfTrajectoryPoints)
   {
      Point3D sphereOrigin = new Point3D(0.0, 0.0, 0.7);
      double sphereRadius = 0.5;
      int numberOfPointsToGenerate = numberOfTrajectoryPoints;
      Point3D[] waypointPositions = SpiralBasedAlgorithm.generatePointsOnSphere(sphereOrigin, sphereRadius, numberOfPointsToGenerate);
      return waypointPositions;
   }

   private Point3D[] generateLineWaypoints(int numberOfTrajectoryPoints)
   {
      Point3D startPoint = new Point3D(-1.0, 0.0, 0.7);
      Point3D endPoint = new Point3D(1.0, 0.0, 0.7);
      Point3D[] waypoints = new Point3D[numberOfTrajectoryPoints];

      for (int i = 0; i < numberOfTrajectoryPoints; i++)
      {
         waypoints[i] = new Point3D();
         waypoints[i].interpolate(startPoint, endPoint, i / (numberOfTrajectoryPoints - 1.0));
      }
      return waypoints;
   }

   public static void main(String[] args)
   {
      new EuclideanTrajectoryPointCalculatorVisualizer();
   }
}
