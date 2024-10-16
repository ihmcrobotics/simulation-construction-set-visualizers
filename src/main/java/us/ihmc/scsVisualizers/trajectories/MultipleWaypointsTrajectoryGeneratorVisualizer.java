package us.ihmc.scsVisualizers.trajectories;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.commons.trajectories.yoVariables.YoPolynomial;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MultipleWaypointsTrajectoryGeneratorVisualizer
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final YoPolynomial simpleTraj;
   private final MultipleWaypointsTrajectoryGenerator traj;
   
   public MultipleWaypointsTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      simpleTraj = new YoPolynomial("simpleTraj", 6, registry);
      double z0 = 0.0;
      double zd0 = 0.0;
      double zdd0 = 0.0;
      double zf = 1.0;
      double zdf = 0.5;
      double zddf = 0.0;
      simpleTraj.setQuintic(0.0, trajectoryTime, z0, zd0, zdd0, zf, zdf, zddf);
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
      scs.addYoRegistry(registry);
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
