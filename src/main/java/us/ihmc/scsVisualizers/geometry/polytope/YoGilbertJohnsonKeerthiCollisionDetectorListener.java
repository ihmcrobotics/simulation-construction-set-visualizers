package us.ihmc.scsVisualizers.geometry.polytope;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.GilbertJohnsonKeerthiCollisionDetectorListener;
import us.ihmc.geometry.polytope.PolytopeVertex;
import us.ihmc.geometry.polytope.SimplexPolytope;
import us.ihmc.geometry.polytope.SupportingVertexHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicTriangle;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class YoGilbertJohnsonKeerthiCollisionDetectorListener implements GilbertJohnsonKeerthiCollisionDetectorListener
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final YoDouble[] lambdas = new YoDouble[4];
   private final YoDouble tripleProduct = new YoDouble("tripleProduct", registry);

   private final YoFramePoint3D[] simplexPoints = new YoFramePoint3D[4];
   private final YoFramePoint3D[] correspondingPointsOnPolytopeA = new YoFramePoint3D[4];
   private final YoFramePoint3D[] correspondingPointsOnPolytopeB = new YoFramePoint3D[4];

   private final int maxNumberOfPolytopePoints = 80;
   private final ArrayList<YoFramePoint3D> polytopeAPoints = new ArrayList<>();
   private final ArrayList<YoFramePoint3D> polytopeBPoints = new ArrayList<>();

   private final int maxNumberOfPolytopeEdges = 200;
   private final ArrayList<YoGraphicLineSegment> polytopeAEdgesViz = new ArrayList<>();
   private final ArrayList<YoGraphicLineSegment> polytopeBEdgesViz = new ArrayList<>();

   private final YoDouble numberOfSimplexPoints = new YoDouble("numberOfSimplexPoints", registry);

   private final YoGraphicTriangle[] simplexTriangles = new YoGraphicTriangle[4];
   private final YoGraphicTriangle[] polytopeATriangles = new YoGraphicTriangle[4];
   private final YoGraphicTriangle[] polytopeBTriangles = new YoGraphicTriangle[4];

   //   private final YoFrameConvexPolygon2d convexPolygon;

   private final YoFramePoint3D closestPointOnSimplex;
   private final YoFramePoint3D closestPointOnA;
   private final YoFramePoint3D closestPointOnB;

   private final YoDouble distanceSimplexToOrigin = new YoDouble("distanceSimplexToOrigin", registry);
   private final YoDouble distanceBetweenClosestPoints = new YoDouble("distanceBetweenClosestPoints", registry);

   private final AppearanceDefinition[] simplexPointAppearances = new AppearanceDefinition[] { YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Blue(), YoAppearance.Black() };
   private final AppearanceDefinition[] simplexTirangleAppearances = new AppearanceDefinition[] { YoAppearance.Orange(), YoAppearance.Purple(), YoAppearance.Coral(), YoAppearance.CornflowerBlue() };

   private final SimulationConstructionSet scs;

   public YoGilbertJohnsonKeerthiCollisionDetectorListener()
   {
      AppearanceDefinition polytopeAAppearance = YoAppearance.DarkBlue();
      AppearanceDefinition polytopeBAppearance = YoAppearance.DarkGreen();

      for (int i = 0; i < maxNumberOfPolytopePoints; i++)
      {
         YoFramePoint3D polytopeAPoint = new YoFramePoint3D("polytopeA" + i, ReferenceFrame.getWorldFrame(), registry);
         polytopeAPoint.setToNaN();
         polytopeAPoints.add(polytopeAPoint);
         YoGraphicPosition polytopeAPointViz = new YoGraphicPosition("polytopeA" + i, polytopeAPoint, 0.03, polytopeAAppearance);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeAPoints", polytopeAPointViz);

         YoFramePoint3D polytopeBPoint = new YoFramePoint3D("polytopeB" + i, ReferenceFrame.getWorldFrame(), registry);
         polytopeBPoint.setToNaN();
         polytopeBPoints.add(polytopeBPoint);
         YoGraphicPosition polytopeBPointViz = new YoGraphicPosition("polytopeB" + i, polytopeBPoint, 0.03, polytopeBAppearance);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeBPoints", polytopeBPointViz);
      }

      for (int i = 0; i < maxNumberOfPolytopeEdges; i++)
      {
         YoGraphicLineSegment polytopeAEdgeViz = new YoGraphicLineSegment("polytopeAEdge" + i, "", ReferenceFrame.getWorldFrame(), polytopeAAppearance, registry);
         polytopeAEdgeViz.setToNaN();
         polytopeAEdgesViz.add(polytopeAEdgeViz);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeAEdges", polytopeAEdgeViz);

         YoGraphicLineSegment polytopeBEdgeViz = new YoGraphicLineSegment("polytopeBEdge" + i, "", ReferenceFrame.getWorldFrame(), polytopeBAppearance, registry);
         polytopeBEdgeViz.setToNaN();
         polytopeBEdgesViz.add(polytopeBEdgeViz);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeBEdges", polytopeBEdgeViz);
      }

      for (int i = 0; i < 4; i++)
      {
         simplexPoints[i] = new YoFramePoint3D("simplexPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition simplexPointViz = new YoGraphicPosition("simplexPoint" + i, simplexPoints[i], 0.1, simplexPointAppearances[i]);
         yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", simplexPointViz);

         correspondingPointsOnPolytopeA[i] = new YoFramePoint3D("correspondingPointOnPolytopeA" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition correspondingPointOnPolytopeAViz = new YoGraphicPosition("correspondingPointOnPolytopeA" + i, correspondingPointsOnPolytopeA[i], 0.06, simplexPointAppearances[i]);
         yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", correspondingPointOnPolytopeAViz);

         correspondingPointsOnPolytopeB[i] = new YoFramePoint3D("correspondingPointOnPolytopeB" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition correspondingPointOnPolytopeBViz = new YoGraphicPosition("correspondingPointOnPolytopeB" + i, correspondingPointsOnPolytopeB[i], 0.06, simplexPointAppearances[i]);
         yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", correspondingPointOnPolytopeBViz);

         lambdas[i] = new YoDouble("lambda" + i, registry);

         simplexTriangles[i] = new YoGraphicTriangle("simplexTriangle" + i + "_", simplexTirangleAppearances[i], registry);
         yoGraphicsListRegistry.registerYoGraphic("SimplexTriangles", simplexTriangles[i]);
         simplexTriangles[i].setToNaN();

         polytopeATriangles[i] = new YoGraphicTriangle("polytopeATriangle" + i + "_", simplexTirangleAppearances[i], registry);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeATriangles", polytopeATriangles[i]);
         polytopeATriangles[i].setToNaN();

         polytopeBTriangles[i] = new YoGraphicTriangle("polytopeBTriangle" + i + "_", simplexTirangleAppearances[i], registry);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeBTriangles", polytopeBTriangles[i]);
         polytopeBTriangles[i].setToNaN();

         yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(simplexTriangles[i]);
         yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(polytopeATriangles[i]);
         yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(polytopeBTriangles[i]);
      }

      //      convexPolygon = new YoFrameConvexPolygon2d("convexPolygon", ReferenceFrame.getWorldFrame(), 6, registry);
      //      ConvexPolygon2d polygon = new ConvexPolygon2d(new double[][]{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
      //      convexPolygon.setConvexPolygon2d(polygon);
      //      YoGraphicPolygon graphicPolygon = new YoGraphicPolygon("polygon", convexPolygon, "foo", "bar", registry, 1.0, YoAppearance.Purple());
      //      yoGraphicsListRegistry.registerYoGraphic("SimplexTriangles", graphicPolygon);
      //      yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(graphicPolygon);

      closestPointOnSimplex = new YoFramePoint3D("closestPointOnSimplex", ReferenceFrame.getWorldFrame(), registry);
      AppearanceDefinition closestPointAppearance = YoAppearance.Gold();
      closestPointAppearance.setTransparency(0.5);
      YoGraphicPosition closestPointOnSimplexViz = new YoGraphicPosition("closestPointOnSimplex", closestPointOnSimplex, 0.14, closestPointAppearance);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", closestPointOnSimplexViz);

      closestPointOnA = new YoFramePoint3D("closestPointOnA", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition closestPointOnAViz = new YoGraphicPosition("closestPointOnAViz", closestPointOnA, 0.12, closestPointAppearance);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", closestPointOnAViz);

      closestPointOnB = new YoFramePoint3D("closestPointOnB", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition closestPointOnBViz = new YoGraphicPosition("closestPointOnBViz", closestPointOnB, 0.12, closestPointAppearance);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", closestPointOnBViz);

      YoGraphicCoordinateSystem originViz = new YoGraphicCoordinateSystem("origin", "", registry, true, 1.0);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", originViz);

      scs = new SimulationConstructionSet(new Robot("GJKVisualizer"));
      scs.setDT(1.0, 1);
      scs.getRootRegistry().addChild(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.updateAndTick();

      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   @Override
   public void checkingIfPolytopesAreColliding(SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB)
   {
      clearAllGraphics();
      scs.setTime(0.0);

      drawPolytopeVertices(polytopeA, polytopeAPoints);
      drawPolytopeVertices(polytopeB, polytopeBPoints);

      drawPolytopeEdges(polytopeA, polytopeAEdgesViz);
      drawPolytopeEdges(polytopeB, polytopeBEdgesViz);

      scs.tickAndUpdate();
   }

   private void clearAllGraphics()
   {
      clearYoGraphicTriangles(simplexTriangles);
      clearYoGraphicTriangles(polytopeATriangles);
      clearYoGraphicTriangles(polytopeBTriangles);

      closestPointOnSimplex.setToNaN();
      closestPointOnA.setToNaN();
      closestPointOnB.setToNaN();

      clearYoFramePoints(simplexPoints);
      clearYoFramePoints(correspondingPointsOnPolytopeA);
      clearYoFramePoints(correspondingPointsOnPolytopeB);

      clearYoFramePoints(polytopeAPoints);
      clearYoFramePoints(polytopeBPoints);

      clearYoGraphicLineSegments(polytopeAEdgesViz);
      clearYoGraphicLineSegments(polytopeBEdgesViz);
   }

   private void clearYoGraphicLineSegments(ArrayList<YoGraphicLineSegment> lineSegments)
   {
      for (YoGraphicLineSegment lineSegment : lineSegments)
      {
         lineSegment.setToNaN();
      }
   }

   private void clearYoFramePoints(ArrayList<YoFramePoint3D> yoFramePoints)
   {
      for (YoFramePoint3D yoFramePoint : yoFramePoints)
      {
         yoFramePoint.setToNaN();
      }
   }

   private void clearYoFramePoints(YoFramePoint3D[] yoFramePoints)
   {
      for (YoFramePoint3D yoFramePoint : yoFramePoints)
      {
         yoFramePoint.setToNaN();
      }
   }

   private void clearYoGraphicTriangles(YoGraphicTriangle[] triangles)
   {
      for (int i = 0; i < triangles.length; i++)
      {
         triangles[i].setToNaN();
      }
   }

   private static void drawPolytopeVertices(SupportingVertexHolder polytope, ArrayList<YoFramePoint3D> polytopeVertcesForViz)
   {
      if (polytope instanceof ConvexPolytope)
      {
         ConvexPolytope convexPolytope = (ConvexPolytope) polytope;
         
         int numberOfVertices = convexPolytope.getNumberOfVertices();

         for (int i = 0; i < numberOfVertices; i++)
         {
            PolytopeVertex vertex = convexPolytope.getVertex(i);
            polytopeVertcesForViz.get(i).set(vertex.getPosition());
         }
      }
   }

   private static void drawPolytopeEdges(SupportingVertexHolder polytope, ArrayList<YoGraphicLineSegment> polytopeEdgesViz)
   {
      if (polytope instanceof ConvexPolytope)
      {
         ConvexPolytope convexPolytope = (ConvexPolytope) polytope;

         int numberOfEdges = convexPolytope.getNumberOfEdges();
         ArrayList<PolytopeVertex[]> edges = convexPolytope.getEdges();

         for (int i = 0; i < numberOfEdges; i++)
         {
            PolytopeVertex[] edge = edges.get(i);
            YoGraphicLineSegment polytopeEdgeViz = polytopeEdgesViz.get(i);
            polytopeEdgeViz.setStartAndEnd(edge[0].getPosition(), edge[1].getPosition());
         }
      }
   }

   @Override
   public void addedVertexToSimplex(SimplexPolytope simplex, Point3D vertexOnSimplex, Point3D vertexOnA, Point3D vertexOnB)
   {
      updateSimplexViz(simplex);
      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   @Override
   public void foundClosestPointOnSimplex(SimplexPolytope simplex, Point3D closestPointToOrigin)
   {
      updateSimplexViz(simplex);
      updateLambdas(simplex);

      closestPointOnSimplex.set(closestPointToOrigin);
      Point3D closestPointOnA = new Point3D();
      Point3D closestPointOnB = new Point3D();
      simplex.getClosestPointsOnAAndB(closestPointOnA, closestPointOnB);

      distanceSimplexToOrigin.set(closestPointToOrigin.distance(new Point3D(0.0, 0.0, 0.0)));
      distanceBetweenClosestPoints.set(closestPointOnA.distance(closestPointOnB));

      this.closestPointOnA.set(closestPointOnA);
      this.closestPointOnB.set(closestPointOnB);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   @Override
   public void foundCollision(SimplexPolytope simplex, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
   }

   @Override
   public void foundSupportPoints(SimplexPolytope simplex, Point3D supportingPointOnA, Point3D supportingPointOnB, Vector3D supportPointOnSimplex)
   {
   }

   @Override
   public void computeVDotPAndPercentCloser(double vDotP, double percentCloser)
   {
   }

   @Override
   public void metStoppingConditionForNoIntersection(double vDotP, double percentCloser, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
   }

   @Override
   public void tooManyIterationsStopping(SimplexPolytope simplex, Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
   }

   @Override
   public void metStoppingConditionForNoIntersection(Point3D pointOnAToPack, Point3D pointOnBToPack)
   {
   }

   private void updateSimplexViz(SimplexPolytope simplex)
   {
      int numberOfPoints = simplex.getNumberOfPoints();
      numberOfSimplexPoints.set(numberOfPoints);

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D simplexPoint = simplex.getPoint(i);
         simplexPoints[i].set(simplexPoint);

         Point3D correspondingPointOnPolytopeA = simplex.getCorrespondingPointOnPolytopeA(simplexPoint);
         correspondingPointsOnPolytopeA[i].set(correspondingPointOnPolytopeA);

         Point3D correspondingPointOnPolytopeB = simplex.getCorrespondingPointOnPolytopeB(simplexPoint);
         correspondingPointsOnPolytopeB[i].set(correspondingPointOnPolytopeB);
      }
      for (int i = numberOfPoints; i < 4; i++)
      {
         simplexPoints[i].setToNaN();
      }

      tripleProduct.set(simplex.computeTripleProductIfTetragon());

      if (numberOfPoints < 3)
      {
         this.simplexTriangles[0].setToNaN();
         this.simplexTriangles[1].setToNaN();
         this.simplexTriangles[2].setToNaN();
         this.simplexTriangles[3].setToNaN();

         this.polytopeATriangles[0].setToNaN();
         this.polytopeATriangles[1].setToNaN();
         this.polytopeATriangles[2].setToNaN();
         this.polytopeATriangles[3].setToNaN();

         this.polytopeBTriangles[0].setToNaN();
         this.polytopeBTriangles[1].setToNaN();
         this.polytopeBTriangles[2].setToNaN();
         this.polytopeBTriangles[3].setToNaN();

         //         ConvexPolygon2d polygon = new ConvexPolygon2d(new double[][]{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
         //         convexPolygon.setConvexPolygon2d(polygon);
      }

      if (numberOfPoints == 3)
      {
         this.simplexTriangles[1].setToNaN();
         this.simplexTriangles[2].setToNaN();
         this.simplexTriangles[3].setToNaN();

         this.polytopeATriangles[1].setToNaN();
         this.polytopeATriangles[2].setToNaN();
         this.polytopeATriangles[3].setToNaN();

         this.polytopeBTriangles[1].setToNaN();
         this.polytopeBTriangles[2].setToNaN();
         this.polytopeBTriangles[3].setToNaN();

         Point3D pointOne = simplex.getPoint(0);
         Point3D pointTwo = simplex.getPoint(1);
         Point3D pointThree = simplex.getPoint(2);

         this.simplexTriangles[0].updatePoints(pointOne, pointTwo, pointThree);
         this.polytopeATriangles[0].updatePoints(simplex.getCorrespondingPointOnPolytopeA(pointOne), simplex.getCorrespondingPointOnPolytopeA(pointTwo), simplex.getCorrespondingPointOnPolytopeA(pointThree));
         this.polytopeBTriangles[0].updatePoints(simplex.getCorrespondingPointOnPolytopeB(pointOne), simplex.getCorrespondingPointOnPolytopeB(pointTwo), simplex.getCorrespondingPointOnPolytopeB(pointThree));

         //         ConvexPolygon2d polygon = new ConvexPolygon2d(new double[][]{{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}});
         //         convexPolygon.setConvexPolygon2d(polygon);
      }

      if (numberOfPoints == 4)
      {
         Point3D pointOne = simplex.getPoint(0);
         Point3D pointTwo = simplex.getPoint(1);
         Point3D pointThree = simplex.getPoint(2);
         Point3D pointFour = simplex.getPoint(3);

         this.simplexTriangles[0].updatePoints(pointOne, pointTwo, pointThree);
         this.simplexTriangles[1].updatePoints(pointOne, pointTwo, pointFour);
         this.simplexTriangles[2].updatePoints(pointOne, pointThree, pointFour);
         this.simplexTriangles[3].updatePoints(pointTwo, pointThree, pointFour);

         this.polytopeATriangles[0].updatePoints(simplex.getCorrespondingPointOnPolytopeA(pointOne), simplex.getCorrespondingPointOnPolytopeA(pointTwo), simplex.getCorrespondingPointOnPolytopeA(pointThree));
         this.polytopeATriangles[1].updatePoints(simplex.getCorrespondingPointOnPolytopeA(pointOne), simplex.getCorrespondingPointOnPolytopeA(pointTwo), simplex.getCorrespondingPointOnPolytopeA(pointFour));
         this.polytopeATriangles[2].updatePoints(simplex.getCorrespondingPointOnPolytopeA(pointOne), simplex.getCorrespondingPointOnPolytopeA(pointThree), simplex.getCorrespondingPointOnPolytopeA(pointFour));
         this.polytopeATriangles[3].updatePoints(simplex.getCorrespondingPointOnPolytopeA(pointTwo), simplex.getCorrespondingPointOnPolytopeA(pointThree), simplex.getCorrespondingPointOnPolytopeA(pointFour));

         this.polytopeBTriangles[0].updatePoints(simplex.getCorrespondingPointOnPolytopeB(pointOne), simplex.getCorrespondingPointOnPolytopeB(pointTwo), simplex.getCorrespondingPointOnPolytopeB(pointThree));
         this.polytopeBTriangles[1].updatePoints(simplex.getCorrespondingPointOnPolytopeB(pointOne), simplex.getCorrespondingPointOnPolytopeB(pointTwo), simplex.getCorrespondingPointOnPolytopeB(pointFour));
         this.polytopeBTriangles[2].updatePoints(simplex.getCorrespondingPointOnPolytopeB(pointOne), simplex.getCorrespondingPointOnPolytopeB(pointThree), simplex.getCorrespondingPointOnPolytopeB(pointFour));
         this.polytopeBTriangles[3].updatePoints(simplex.getCorrespondingPointOnPolytopeB(pointTwo), simplex.getCorrespondingPointOnPolytopeB(pointThree), simplex.getCorrespondingPointOnPolytopeB(pointFour));

         //         ConvexPolygon2d polygon = new ConvexPolygon2d(new double[][]{{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}, {0.0, 3.0}});
         //         convexPolygon.setConvexPolygon2d(polygon);
      }
   }

   private void updateLambdas(SimplexPolytope simplex)
   {
      int numberOfPoints = simplex.getNumberOfPoints();

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D simplexPoint = simplex.getPoint(i);
         lambdas[i].set(simplex.getLambda(simplexPoint));
      }
   }

   public void cropDataBuffer()
   {
      scs.cropBuffer();
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

}
