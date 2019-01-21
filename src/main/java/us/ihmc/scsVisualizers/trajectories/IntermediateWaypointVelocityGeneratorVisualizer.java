package us.ihmc.scsVisualizers.trajectories;

import java.util.ArrayList;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.math.trajectories.IntermediateWaypointVelocityGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class IntermediateWaypointVelocityGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 3.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final MultipleWaypointsPositionTrajectoryGenerator traj;
   private final StraightLinePositionTrajectoryGenerator simpleTraj;

   public IntermediateWaypointVelocityGeneratorVisualizer()
   {
      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(trajectoryTime);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(new FramePoint3D(worldFrame, 1.0, 0.0, 1.0));
      PositionProvider finalPositionProvider = new ConstantPositionProvider(new FramePoint3D(worldFrame, 0.2, 1.0, 0.4));
      simpleTraj = new StraightLinePositionTrajectoryGenerator("simpleTraj", worldFrame, trajectoryTimeProvider, initialPositionProvider, finalPositionProvider,
            registry);
      simpleTraj.initialize();

      traj = new MultipleWaypointsPositionTrajectoryGenerator("testedTraj", 200, worldFrame, registry);
      traj.clear();

      IntermediateWaypointVelocityGenerator velocityGen;

      int numberOfWaypoints = 11;

      FramePoint3D waypointPosition = new FramePoint3D();
      FrameVector3D waypointVelocity = new FrameVector3D();
      Vector3D currentPoint = new Vector3D();

      ArrayList<Vector3D> waypoints = new ArrayList<>();
      ArrayList<Double> time = new ArrayList<>();

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         simpleTraj.getPosition(waypointPosition);

         currentPoint.set(waypointPosition);
         waypoints.add(new Vector3D(currentPoint));
         time.add(timeAtWaypoint);
      }

      velocityGen = new IntermediateWaypointVelocityGenerator(time, waypoints);
      ArrayList<Vector3D> xDot = velocityGen.calculateVelocities();

      double timeAtWaypoint = 0;
      simpleTraj.compute(timeAtWaypoint);
      simpleTraj.getPosition(waypointPosition);
      waypointVelocity.set(0, 0, 0);
      traj.appendWaypoint(timeAtWaypoint, waypointPosition, waypointVelocity);

      for (int i = 1; i < numberOfWaypoints - 1; i++)
      {
         timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
         simpleTraj.compute(timeAtWaypoint);
         simpleTraj.getPosition(waypointPosition);
         waypointVelocity.set(xDot.get(i - 1));

         traj.appendWaypoint(timeAtWaypoint, waypointPosition, waypointVelocity);
      }

      simpleTraj.compute(trajectoryTime);
      simpleTraj.getPosition(waypointPosition);
      waypointVelocity.set(0, 0, 0);
      traj.appendWaypoint(trajectoryTime, waypointPosition, waypointVelocity);

      //      for (int i = 0; i < numberOfWaypoints; i++)
      //      {
      //         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
      //         currentPoint = new Vector3d(3 * (i * i) * 0.01, 0.2 * i, 0.1*i *i);
      //
      //         waypoints.add(new Vector3d(currentPoint));
      //         time.add(timeAtWaypoint);
      //      }
      //
      //      ArrayList<Double> subTime = new ArrayList(time.subList(1, numberOfWaypoints));
      //      ArrayList<Vector3d> subWayPoints = new ArrayList(waypoints.subList(1, numberOfWaypoints));
      //
      //      velocityGen = new IntermediateWaypointVelocityGenerator(subTime, subWayPoints);
      //      ArrayList<Vector3d> xDot = velocityGen.calculateVelocities();
      //
      //      waypointVelocity.set(0,0,0);
      //      waypointPosition.set(waypoints.get(0));
      //
      //      traj.appendWaypoint(0, waypointPosition, waypointVelocity);
      //
      //      for (int i = 1; i < numberOfWaypoints-1; i++)
      //      {
      //         double timeAtWaypoint = i * trajectoryTime / (numberOfWaypoints - 1.0);
      //         waypointVelocity.set(xDot.get(i-1));
      //         waypointPosition.set(waypoints.get(i));
      //
      //         traj.appendWaypoint(timeAtWaypoint, waypointPosition, waypointVelocity);
      //      }
      //
      //      waypointVelocity.set(0,0,0);
      //      waypointPosition.set(waypoints.get(numberOfWaypoints-1));
      //
      //      traj.appendWaypoint(trajectoryTime, waypointPosition, waypointVelocity);

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
      new IntermediateWaypointVelocityGeneratorVisualizer();
   }
}
