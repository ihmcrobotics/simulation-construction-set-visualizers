package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;
import java.util.function.Supplier;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.OneDoFTrajectoryPointCalculator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.YoFrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.OneDoFTrajectoryPointList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TrajectoryPoint1DCalculatorVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final MultipleWaypointsTrajectoryGenerator traj;
   private final OneDoFTrajectoryPointCalculator calculator = new OneDoFTrajectoryPointCalculator();

   private final YoFramePoint3D currentPositionViz = new YoFramePoint3D("currentPositionViz", worldFrame, registry);
   private final RecyclingArrayList<YoFrameEuclideanTrajectoryPoint> waypointsViz;

   public TrajectoryPoint1DCalculatorVisualizer()
   {
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      int numberOfWaypoints = 5;

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
      waypointsViz = new RecyclingArrayList<>(numberOfWaypoints, builder);

      yoGraphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("currentPositionViz", currentPositionViz, 0.05, YoAppearance.Red()));

      Random random = new Random(519651L);
      double position = 1.0;

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         position += RandomNumbers.nextDouble(random, 0.3);
//         position = 0.5 * Math.sin(2.0 * Math.PI * i * trajectoryTime / (numberOfWaypoints - 1)) + 1.0;
         calculator.appendTrajectoryPoint(position);
      }

      calculator.compute(trajectoryTime);

      OneDoFTrajectoryPointList trajectoryData = calculator.getTrajectoryData();

      traj = new MultipleWaypointsTrajectoryGenerator("traj", calculator.getNumberOfTrajectoryPoints(), registry);
      traj.appendWaypoints(trajectoryData);
      traj.initialize();

      double xStart = -0.5;
      double xEnd = 0.5;

      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double alpha = trajectoryData.getTrajectoryPoint(i).getTime() / trajectoryTime;
         double x = (1.0 - alpha) * xStart + alpha * xEnd;
         Point3D position3d = new Point3D(x, 0.0, trajectoryData.getTrajectoryPoint(i).getPosition());
         Vector3D linearVelocity3d = new Vector3D(0.0, 0.0, trajectoryData.getTrajectoryPoint(i).getVelocity());
         waypointsViz.add().set(trajectoryData.getTrajectoryPoint(i).getTime(), position3d, linearVelocity3d);
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

         double alpha = t / trajectoryTime;
         double currentX = (1.0 - alpha) * xStart + alpha * xEnd;
         currentPositionViz.set(currentX, 0.0, traj.getValue());

         scs.setTime(t);
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
