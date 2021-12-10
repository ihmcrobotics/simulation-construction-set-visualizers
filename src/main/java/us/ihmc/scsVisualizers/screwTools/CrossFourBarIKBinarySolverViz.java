package us.ihmc.scsVisualizers.screwTools;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.mecano.fourBar.CrossFourBarJointIKBinarySolver;
import us.ihmc.mecano.fourBar.CrossFourBarJointIKSolver;
import us.ihmc.mecano.fourBar.FourBar;
import us.ihmc.mecano.fourBar.FourBarAngle;
import us.ihmc.mecano.fourBar.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;
import us.ihmc.mecano.fourBar.FourBarVertex;
import us.ihmc.mecano.multiBodySystem.CrossFourBarJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public class CrossFourBarIKBinarySolverViz
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public CrossFourBarIKBinarySolverViz()
   {
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters(true, 10000 + 1);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("dummy"), parameters);

      YoDouble exactDAB = new YoDouble("exactDAB", scs.getRootRegistry());
      YoDouble solverDAB = new YoDouble("solverDAB", scs.getRootRegistry());
      YoDouble errorDAB = new YoDouble("errorDAB", scs.getRootRegistry());
      YoDouble theta = new YoDouble("theta", scs.getRootRegistry());

      CrossFourBarJoint crossFourBarJoint = createKnownCrossFourBarJoint2("blop", 0, 1.0e-3);

      CrossFourBarJointIKSolver ikSolver = crossFourBarJoint.getIKSolver();
      FourBar fourBar = crossFourBarJoint.getFourBarFunction().getFourBar();
      FourBarVertex vertexA = fourBar.getVertexA();
      double minDAB = vertexA.getMinAngle();
      double maxDAB = vertexA.getMaxAngle();

      double rangeDAB = maxDAB - minDAB;
      double deltaDAB = rangeDAB / 10000.0;
      FourBarToJointConverter[] converters = crossFourBarJoint.getFourBarFunction().getConverters();

      scs.startOnAThread();

      for (double DAB = minDAB; DAB <= maxDAB; DAB += deltaDAB)
      {
         fourBar.update(FourBarAngle.DAB, DAB);
         exactDAB.set(DAB);
         theta.set(converters[0].toJointAngle(DAB) + converters[3].toJointAngle(fourBar.getAngleCDA()));
         solverDAB.set(converters[0].toFourBarInteriorAngle(ikSolver.solve(theta.getValue(), vertexA)));
         errorDAB.set(Math.abs(exactDAB.getValue() - solverDAB.getValue()));
         scs.tickAndUpdate();
      }
   }

   private static CrossFourBarJoint createKnownCrossFourBarJoint2(String name, int actuatedJointIndex, double solverTolerance)
   {
      Point2D A = new Point2D(0.227, 0.1);
      Point2D B = new Point2D(0.227, -0.1);
      Point2D C = new Point2D(0.427, 0.1);
      Point2D D = new Point2D(0.427, -0.1);
      return createCrossFourBarJoint(name, A, B, C, D, actuatedJointIndex, solverTolerance);
   }

   private static CrossFourBarJoint createCrossFourBarJoint(String name,
                                                            Point2DReadOnly A,
                                                            Point2DReadOnly B,
                                                            Point2DReadOnly C,
                                                            Point2DReadOnly D,
                                                            int actuatedJointIndex,
                                                            double solverTolerance)
   {
      RigidBody rootBody = new RigidBody("root", worldFrame);
      RigidBodyTransform transformAToPredecessor = new RigidBodyTransform(new Quaternion(), new Vector3D(A));
      RigidBodyTransform transformBToPredecessor = new RigidBodyTransform(new Quaternion(), new Vector3D(B));
      Vector2D AD = new Vector2D();
      AD.sub(D, A);
      Vector2D BC = new Vector2D();
      BC.sub(C, B);
      RigidBodyTransform transformCToB = new RigidBodyTransform(new Quaternion(), new Vector3D(BC));
      RigidBodyTransform transformDToA = new RigidBodyTransform(new Quaternion(), new Vector3D(AD));
      CrossFourBarJoint joint = new CrossFourBarJoint(name,
                                                      rootBody,
                                                      "jointA",
                                                      "jointB",
                                                      "jointC",
                                                      "jointD",
                                                      "bodyDA",
                                                      "bodyBC",
                                                      transformAToPredecessor,
                                                      transformBToPredecessor,
                                                      transformDToA,
                                                      transformCToB,
                                                      null,
                                                      null,
                                                      0,
                                                      0,
                                                      null,
                                                      null,
                                                      actuatedJointIndex,
                                                      3,
                                                      Axis3D.Z);
      joint.setIKSolver(new CrossFourBarJointIKBinarySolver(solverTolerance));
      new RigidBody("bodyCD", joint, new Matrix3D(), 0.0, new RigidBodyTransform());
      return joint;
   }

   public static void main(String[] args)
   {
      new CrossFourBarIKBinarySolverViz();
   }
}
