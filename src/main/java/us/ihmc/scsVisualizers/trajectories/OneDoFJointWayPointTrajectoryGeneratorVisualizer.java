package us.ihmc.scsVisualizers.trajectories;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.trajectories.OneDoFJointWayPointTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.providers.YoVariableDoubleProvider;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.commons.thread.ThreadTools;

public class OneDoFJointWayPointTrajectoryGeneratorVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 1.0;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final OneDoFJointWayPointTrajectoryGenerator traj;
   
   private final YoDouble[] positions;
   private final YoVariableDoubleProvider trajectoryTimeProvider;

   private final YoDouble currentPosition = new YoDouble("currentPosition", registry);
   private final YoDouble currentVelocity = new YoDouble("currentVelocity", registry);
   private final YoDouble currentAcceleration = new YoDouble("currentAcceleration", registry);

   private final Random random = new Random(21612L);
   
   public OneDoFJointWayPointTrajectoryGeneratorVisualizer()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      positions = new YoDouble[5];
      
      for (int i = 0; i < positions.length; i++)
      {
         positions[i] = new YoDouble("desiredPosition" + Integer.toString(i), registry);
         positions[i].set(i * 10.0 * random.nextDouble());
      }

      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      RevoluteJoint joint = ScrewTools.addRevoluteJoint("joint", elevator, new RigidBodyTransform(), new Vector3D(0.0, 0.0, 1.0));
      trajectoryTimeProvider = new YoVariableDoubleProvider("trajectoryTime", registry);
      trajectoryTimeProvider.set(trajectoryTime);
      traj = new OneDoFJointWayPointTrajectoryGenerator("Blop", joint, trajectoryTimeProvider, 30, registry);
      double[] desiredPositions = new double[positions.length];
      for (int i = 0; i < positions.length; i++)
         desiredPositions[i] = positions[i].getDoubleValue();
      traj.setDesiredPositions(desiredPositions);
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
         currentPosition.set(traj.getValue());
         currentVelocity.set(traj.getVelocity());
         currentAcceleration.set(traj.getAcceleration());

         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new OneDoFJointWayPointTrajectoryGeneratorVisualizer();
   }
}
