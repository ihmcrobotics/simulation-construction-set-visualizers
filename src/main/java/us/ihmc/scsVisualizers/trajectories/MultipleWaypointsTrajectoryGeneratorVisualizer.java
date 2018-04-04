package us.ihmc.scsVisualizers.trajectories;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.QuinticPolynomialTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class MultipleWaypointsTrajectoryGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final QuinticPolynomialTrajectoryGenerator simpleTraj;
   private final MultipleWaypointsTrajectoryGenerator traj;
   
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   public MultipleWaypointsTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      trajectoryTimeProvider = new YoVariableDoubleProvider("trajectoryTime", registry);
      trajectoryTimeProvider.set(trajectoryTime);

      DoubleProvider initialPositionProvider = new ConstantDoubleProvider(0.0);
      DoubleProvider initialVelocityProvider = new ConstantDoubleProvider(0.0);
      DoubleProvider finalPositionProvider = new ConstantDoubleProvider(1.0);
      DoubleProvider finalVelocityProvider = new ConstantDoubleProvider(0.5);
      simpleTraj = new QuinticPolynomialTrajectoryGenerator("simpleTraj", initialPositionProvider, initialVelocityProvider, finalPositionProvider, finalVelocityProvider, trajectoryTimeProvider, registry);
      simpleTraj.initialize();

      traj = new MultipleWaypointsTrajectoryGenerator("testedTraj", 15, registry);
      traj.clear();
      int numberOfWaypoints = 11;
      for (int i = 0; i < numberOfWaypoints; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         double waypointPosition = simpleTraj.getValue();
         double waypointVelocity = simpleTraj.getVelocity();
         traj.appendWaypoint(timeAtWaypoint, waypointPosition, waypointVelocity);
      }
      traj.initialize();
      
      
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
         simpleTraj.compute(t);

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new MultipleWaypointsTrajectoryGeneratorVisualizer();
   }
}
