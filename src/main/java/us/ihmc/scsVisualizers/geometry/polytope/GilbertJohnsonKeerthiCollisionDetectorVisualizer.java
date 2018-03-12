package us.ihmc.scsVisualizers.geometry.polytope;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.ConvexPolytopeConstructor;
import us.ihmc.geometry.polytope.ConvexPolytopeFromExpandingPolytopeEntryGenerator;
import us.ihmc.geometry.polytope.CylinderSupportingVertexHolder;
import us.ihmc.geometry.polytope.ExpandingPolytopeAlgorithm;
import us.ihmc.geometry.polytope.ExpandingPolytopeEntry;
import us.ihmc.geometry.polytope.ExpandingPolytopeEntryFromSimpleMeshGenerator;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.geometry.polytope.IcoSphereCreator;
import us.ihmc.geometry.polytope.SimpleTriangleMesh;
import us.ihmc.geometry.polytope.SimplexPolytope;
import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class GilbertJohnsonKeerthiCollisionDetectorVisualizer
{
   private final YoGilbertJohnsonKeerthiCollisionDetectorListener listener;
   private final GilbertJohnsonKeerthiCollisionDetector detector;

   private final ExpandingPolytopeAlgorithm expandingPolytopeAlgorithm;
   private final YoExpandingPolytopeAlgorithmListener expandingPolytypeListener;

   public GilbertJohnsonKeerthiCollisionDetectorVisualizer()
   {
      listener = new YoGilbertJohnsonKeerthiCollisionDetectorListener();
      detector = new GilbertJohnsonKeerthiCollisionDetector();
      detector.setGilbertJohnsonKeerthiCollisionDetectorListener(listener);

      double epsilonRelative = 1e-4;
      expandingPolytopeAlgorithm = new ExpandingPolytopeAlgorithm(epsilonRelative);
      expandingPolytypeListener = new YoExpandingPolytopeAlgorithmListener();
      expandingPolytopeAlgorithm.setExpandingPolytopeAlgorithmListener(expandingPolytypeListener);
   }

   private void visualizeCollisionDetection(SupportingVertexHolder polytopeOne, SupportingVertexHolder polytopeTwo)
   {
      Point3D closestPointOnA = new Point3D();
      Point3D closetsPointOnB = new Point3D();

      boolean arePolytopesColliding = detector.arePolytopesColliding(polytopeOne, polytopeTwo, closestPointOnA, closetsPointOnB);

      if (arePolytopesColliding)
      {
         SimplexPolytope simplexPolytope = detector.getSimplex();

         expandingPolytopeAlgorithm.setPolytopes(simplexPolytope, polytopeOne, polytopeTwo);
         Vector3D separatingVector = new Vector3D();
         expandingPolytopeAlgorithm.computeExpandedPolytope(separatingVector, closestPointOnA, closetsPointOnB);
      }
   }

   private void cropDataBuffer()
   {
      listener.cropDataBuffer();
      expandingPolytypeListener.cropDataBuffer();
   }

   private static void visualizeAPointInsideAPolytope(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      double halfLengthX = 0.5;
      double halfWidthY = 0.5;
      double halfHeightZ = 0.5;

      ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
      ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructSinglePointPolytope(new Point3D());

      RigidBodyTransform transformOne = new RigidBodyTransform();

      transformOne.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(0.1, 0.2, 0.3);

      polytopeOne.applyTransform(transformOne);
      visualizer.visualizeCollisionDetection(polytopeOne, polytopeTwo);
   }

   private static void visualizeACoupleCubes(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      double halfLengthX = 0.4;
      double halfWidthY = 0.5;
      double halfHeightZ = 0.6;

      ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
      ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      transformOne.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(2.0, 0.0, 0.0);

      transformTwo.setRotationEulerAndZeroTranslation(Math.PI / 4.0, Math.PI / 7.0, Math.PI / 9.0);
      transformTwo.setTranslation(3.5, 0.0, 0.0);

      polytopeOne.applyTransform(transformOne);
      polytopeTwo.applyTransform(transformTwo);

      visualizer.visualizeCollisionDetection(polytopeOne, polytopeTwo);
   }

   private static void visualizeACoupleCollidingCubes(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      double halfLengthX = 0.5;
      double halfWidthY = 0.5;
      double halfHeightZ = 0.5;

      ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
      ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      transformOne.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(2.0, 0.0, 0.0);

      transformTwo.setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI / 12.0);
      transformTwo.setTranslation(2.8, 0.1, 0.1);

      polytopeOne.applyTransform(transformOne);
      polytopeTwo.applyTransform(transformTwo);

      visualizer.visualizeCollisionDetection(polytopeOne, polytopeTwo);
   }

   private static void visualizeACoupleRandomSphereOutlinedPolytopes(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      Random random = new Random(1984L);

      int numberOfPointsOne = 30;
      int numberOfPointsTwo = 10;
      double radius = 1.0;
      double xyzBoundary = 2.0;
      ConvexPolytope polytopeOne = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPointsOne, radius, xyzBoundary);
      ConvexPolytope polytopeTwo = ConvexPolytopeConstructor.constructRandomSphereOutlinedPolytope(random, numberOfPointsTwo, radius, xyzBoundary);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      transformOne.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(2.0, 0.0, 0.0);

      transformTwo.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformTwo.setTranslation(5.0, 0.0, 0.0);

      polytopeOne.applyTransform(transformOne);
      polytopeTwo.applyTransform(transformTwo);

      visualizer.visualizeCollisionDetection(polytopeOne, polytopeTwo);
   }

   private static void visualizeACoupleIcoSpherePolytopes(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      IcoSphereCreator creator = new IcoSphereCreator();
      SimpleTriangleMesh sphereOne = creator.createIcoSphere(0);
      SimpleTriangleMesh sphereTwo = creator.createIcoSphere(1);

      ExpandingPolytopeEntryFromSimpleMeshGenerator generator = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      ExpandingPolytopeEntry expandingPolytopeOne = generator.generateExpandingPolytope(sphereOne);
      ExpandingPolytopeEntry expandingPolytopeTwo = generator.generateExpandingPolytope(sphereTwo);

      ConvexPolytopeFromExpandingPolytopeEntryGenerator generatorTwo = new ConvexPolytopeFromExpandingPolytopeEntryGenerator();

      ConvexPolytope polytopeOne = generatorTwo.generateConvexPolytope(expandingPolytopeOne);
      ConvexPolytope polytopeTwo = generatorTwo.generateConvexPolytope(expandingPolytopeTwo);

      RigidBodyTransform transformOne = new RigidBodyTransform();
      RigidBodyTransform transformTwo = new RigidBodyTransform();

      transformOne.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformOne.setTranslation(4.1, 0.0, 0.0);

      transformTwo.setRotationEulerAndZeroTranslation(0.0, 0.0, 0.0);
      transformTwo.setTranslation(5.0, 0.0, 0.0);

      polytopeOne.applyTransform(transformOne);
      polytopeTwo.applyTransform(transformTwo);

      visualizer.visualizeCollisionDetection(polytopeOne, polytopeTwo);
   }

   private static void visualizeTroublesomeTetragon(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      ConvexPolytope polytopeOne = new ConvexPolytope();

      Point3D pointOne = new Point3D(-1.0, -1.01, -3.01);
      Point3D pointTwo = new Point3D(1.00, 0.98, -3.0);
      Point3D pointThree = new Point3D(-1.00, 0.98, -3.0);
      Point3D pointFour = new Point3D(-0.988, -1.01, -3.0);

      polytopeOne.addVertex(pointOne);
      polytopeOne.addVertex(pointTwo);
      polytopeOne.addVertex(pointThree);
      polytopeOne.addVertex(pointFour);

      ConvexPolytope polytopeTwo = new ConvexPolytope();
      Point3D origin = new Point3D();
      polytopeTwo.addVertex(origin);

      visualizer.visualizeCollisionDetection(polytopeOne, polytopeTwo);
   }

   private static void visualizeTroublesomeCubes(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      double[][] cubeOneVertices = new double[][] {{-0.5903738864022472, 0.04082374023374287, 0.10539748532999185},
            {-0.5778817564514865, -0.04473636929849633, 0.12059633456225372}, {-0.6790574742661268, -0.06840353680005194, 0.07052246269743494},
            {-0.6915496042168874, 0.017156572732187247, 0.055323613465173074}, {-0.5642605772611933, 0.03569432285950272, 0.05505917669100756},
            {-0.5517684473104327, -0.04986578667273647, 0.07025802592326942}, {-0.6529441651250729, -0.07353295417429209, 0.020184154058450643},
            {-0.6654362950758336, 0.0120271553579471, 0.00498530482618878}};

      double[][] cubeTwoVertices = new double[][] {{-100.005, -100.0, -0.005}, {99.995, -100.0, -0.005}, {99.995, 100.0, -0.005}, {-100.005, 100.0, -0.005},
            {-100.005, -100.0, 0.005}, {99.995, -100.0, 0.005}, {99.995, 100.0, 0.005}, {-100.005, 100.0, 0.005}};

      ConvexPolytope cubeOne = ConvexPolytopeConstructor.constructFromVertices(cubeOneVertices);
      ConvexPolytope cubeTwo = ConvexPolytopeConstructor.constructFromVertices(cubeTwoVertices);

      visualizer.visualizeCollisionDetection(cubeOne, cubeTwo);
   }

   private static void visualizeCylinder(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      double radius = 0.2;
      double height = 0.5;
      CylinderSupportingVertexHolder cylinder = new CylinderSupportingVertexHolder(radius, height);
      RigidBodyTransform transformOne = new RigidBodyTransform();
      transformOne.setRotationPitchAndZeroTranslation(Math.PI/12.0);
      transformOne.setTranslation(2.0, 0.0, 0.0);
      cylinder.setTransform(transformOne);

      double halfLengthX = 0.3;
      double halfWidthY = 0.4;
      double halfHeightZ = 0.5;
      ConvexPolytope cube = ConvexPolytopeConstructor.constructBoxWithCenterAtZero(halfLengthX, halfWidthY, halfHeightZ);
      
      RigidBodyTransform transformTwo = new RigidBodyTransform();
      transformTwo.setTranslation(2.6, 0.0, 0.0);
      cube.applyTransform(transformTwo);

      visualizer.visualizeCollisionDetection(cylinder, cube);
      
      SimulationConstructionSet scs = visualizer.getSimulationConstructionSet();
      
      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      AppearanceDefinition cylinderAppearance = YoAppearance.AliceBlue();
      cylinderAppearance.setTransparency(0.5);
      staticLinkGraphics.translate(0.0, 0.0, -height/2.0);
      staticLinkGraphics.transform(transformOne);
      staticLinkGraphics.addCylinder(height, radius, cylinderAppearance);
      scs.addStaticLinkGraphics(staticLinkGraphics );
   }

   private static void visualizeTroublesomeCylinder(GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer)
   {
      double[][] polytopeVertices = new double[][] {{4.7290109868252195, -9.48051959337279, 9.86066553142316},
         {4.992334394846338, -8.508417923723337, 11.746865167550746}, {4.909690197353282, -9.043958943334093, 11.732447362884571},
         {4.227668551037339, -8.116865776445794, 10.880447832028764}, {5.053783588777796, -9.708661418625793, 10.018042440299405},
         {6.191961614893392, -8.684064716610006, 11.01506200303345}};

         ConvexPolytope polytope = ConvexPolytopeConstructor.constructFromVertices(polytopeVertices);

         double radius = 1.4057784173328112;
         double height = 0.807422643821864;

         CylinderSupportingVertexHolder cylinder = new CylinderSupportingVertexHolder(radius, height);
         DenseMatrix64F matrix = new DenseMatrix64F(new double[][] {{-0.04233754, 0.45458570, 0.88969623, 5.84106795},
            {0.36801675, 0.83497813, -0.40911513, -8.76333957}, {-0.92885478, 0.31010218, -0.20264607, 9.64464579}, {0, 0, 0, 1}});

         RigidBodyTransform transform = new RigidBodyTransform(matrix);
         cylinder.setTransform(transform);

         visualizer.visualizeCollisionDetection(polytope, cylinder);

         SimulationConstructionSet scs = visualizer.getSimulationConstructionSet();

         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         AppearanceDefinition cylinderAppearance = YoAppearance.AliceBlue();
         cylinderAppearance.setTransparency(0.5);
         staticLinkGraphics.transform(transform);
         staticLinkGraphics.translate(0.0, 0.0, -height/2.0);
         staticLinkGraphics.addCylinder(height, radius, cylinderAppearance);
         scs.addStaticLinkGraphics(staticLinkGraphics );
   }
   
   private SimulationConstructionSet getSimulationConstructionSet()
   {
      return listener.getSimulationConstructionSet();
   }

   public static void main(String[] args)
   {
      GilbertJohnsonKeerthiCollisionDetectorVisualizer visualizer = new GilbertJohnsonKeerthiCollisionDetectorVisualizer();

      //      visualizeTroublesomeTetragon(visualizer);
      //      visualizeACoupleRandomSphereOutlinedPolytopes(visualizer);

            visualizeACoupleIcoSpherePolytopes(visualizer);

      //      visualizeAPointInsideAPolytope(visualizer);
//            visualizeACoupleCollidingCubes(visualizer);
      //      visualizeACoupleCubes(visualizer);

//      visualizeTroublesomeCubes(visualizer);

//      visualizeCylinder(visualizer);
//      visualizeTroublesomeCylinder(visualizer);

      visualizer.cropDataBuffer();
   }
}
