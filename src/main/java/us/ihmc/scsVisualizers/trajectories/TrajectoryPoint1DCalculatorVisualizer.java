package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPoint1DCalculator;
import us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.OneDoFTrajectoryPointInterface;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;

public class TrajectoryPoint1DCalculatorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final MultipleWaypointsTrajectoryGenerator traj;
   private final TrajectoryPoint1DCalculator calculator = new TrajectoryPoint1DCalculator();

   private final YoFramePoint currentPositionViz = new YoFramePoint("currentPositionViz", worldFrame, registry);
   private final RecyclingArrayList<YoFrameEuclideanTrajectoryPoint> waypointsViz;

   public TrajectoryPoint1DCalculatorVisualizer()
   {
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      int numberOfWaypoints = 5;

      GenericTypeBuilder<YoFrameEuclideanTrajectoryPoint> builder = new GenericTypeBuilder<YoFrameEuclideanTrajectoryPoint>()
      {
         private int i = 0;
         @Override
         public YoFrameEuclideanTrajectoryPoint newInstance()
         {
            String indexAsString = Integer.toString(i++);
            YoFrameEuclideanTrajectoryPoint ret = new YoFrameEuclideanTrajectoryPoint("waypointViz", indexAsString, registry, worldFrame);
            yoGraphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("waypointPosition" + indexAsString, ret.getPosition(), 0.025, YoAppearance.AliceBlue()));
            return ret;
         }
      };
      waypointsViz = new RecyclingArrayList<>(numberOfWaypoints, builder);

      yoGraphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("currentPositionViz", currentPositionViz, 0.05, YoAppearance.Red()));

      Random random = new Random(519651L);
      double position = 1.0;

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         position += RandomNumbers.nextDouble(random, 0.3);
//         position = 0.5 * Math.sin(2.0 * Math.PI * i * trajectoryTime / (numberOfWaypoints - 1)) + 1.0;
         calculator.appendTrajectoryPoint(Double.NaN, position, Double.NaN);
      }

      calculator.computeTrajectoryPointTimes(0.0, trajectoryTime);
      calculator.computeTrajectoryPointVelocities(true);

      RecyclingArrayList<? extends OneDoFTrajectoryPointInterface<?>> waypoints = calculator.getTrajectoryPoints();

      traj = new MultipleWaypointsTrajectoryGenerator("traj", calculator.getNumberOfTrajectoryPoints(), registry);
      traj.appendWaypoints(waypoints);
      traj.initialize();
      
      double xStart = -0.5;
      double xEnd = 0.5;

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double alpha = waypoints.get(i).getTime() / trajectoryTime;
         double x = (1.0 - alpha) * xStart + alpha * xEnd;
         Point3D position3d = new Point3D(x, 0.0, waypoints.get(i).getPosition());
         Vector3D linearVelocity3d = new Vector3D(0.0, 0.0, waypoints.get(i).getVelocity());
         waypointsViz.get(i).set(waypoints.get(i).getTime(), position3d, linearVelocity3d);
      }

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         traj.compute(t);

         double alpha = t / trajectoryTime;
         double currentX = (1.0 - alpha) * xStart + alpha * xEnd;
         currentPositionViz.set(currentX, 0.0, traj.getValue());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
      
   }

   public static void main(String[] args)
   {
      new TrajectoryPoint1DCalculatorVisualizer();
   }
}
