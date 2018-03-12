package us.ihmc.scsVisualizers.geometry.polytope;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.ExpandingPolytopeAlgorithmListener;
import us.ihmc.geometry.polytope.ExpandingPolytopeEdge;
import us.ihmc.geometry.polytope.ExpandingPolytopeEdgeList;
import us.ihmc.geometry.polytope.ExpandingPolytopeEntry;
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
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class YoExpandingPolytopeAlgorithmListener implements ExpandingPolytopeAlgorithmListener
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final YoDouble[] lambdas = new YoDouble[4];
   private final YoDouble tripleProduct = new YoDouble("tripleProduct", registry);

   private final YoFramePoint[] simplexPoints = new YoFramePoint[4];
   private final YoFramePoint[] correspondingPointsOnPolytopeA = new YoFramePoint[4];
   private final YoFramePoint[] correspondingPointsOnPolytopeB = new YoFramePoint[4];

   private final YoFramePoint wYoFramePoint = new YoFramePoint("wYoFramePoint", ReferenceFrame.getWorldFrame(), registry);

   private final int maxNumberOfPolytopePoints = 80;
   private final ArrayList<YoFramePoint> polytopeAPoints = new ArrayList<>();
   private final ArrayList<YoFramePoint> polytopeBPoints = new ArrayList<>();

   private final int maxNumberOfPolytopeEdges = 200;
   private final ArrayList<YoGraphicLineSegment> polytopeAEdgesViz = new ArrayList<>();
   private final ArrayList<YoGraphicLineSegment> polytopeBEdgesViz = new ArrayList<>();

   private final ArrayList<YoGraphicLineSegment> silhouetteEdgesViz = new ArrayList<>();

   private final int maxNumberOfExpandingPolygonTriangles = 120;
   private final ArrayList<YoGraphicTriangle> expandingPolytopeTriangles = new ArrayList<>();
   private final ArrayList<YoGraphicTriangle> polytopeATriangles = new ArrayList<>();
   private final ArrayList<YoGraphicTriangle> polytopeBTriangles = new ArrayList<>();

   private final YoDouble numberOfSimplexPoints = new YoDouble("numberOfSimplexPoints", registry);

   private final YoGraphicTriangle[] simplexTriangles = new YoGraphicTriangle[4];

   private final YoFramePoint closestPointSoFar;
   private final YoFramePoint closestPointOnExpandingPolytope;
   private final YoFramePoint closestPointOnA;
   private final YoFramePoint closestPointOnB;

   private final YoDouble distanceSimplexToOrigin = new YoDouble("distanceSimplexToOrigin", registry);
   private final YoDouble distanceBetweenClosestPoints = new YoDouble("distanceBetweenClosestPoints", registry);

   private final AppearanceDefinition[] simplexPointAppearances = new AppearanceDefinition[] {YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Blue(),
         YoAppearance.Black()};
   private final AppearanceDefinition[] simplexTirangleAppearances = new AppearanceDefinition[] {YoAppearance.Orange(), YoAppearance.Purple(),
         YoAppearance.Coral(), YoAppearance.CornflowerBlue()};

   private final SimulationConstructionSet scs;

   public YoExpandingPolytopeAlgorithmListener()
   {
      AppearanceDefinition polytopeAAppearance = YoAppearance.DarkBlue();
      AppearanceDefinition polytopeBAppearance = YoAppearance.DarkGreen();
      AppearanceDefinition silhouetteAppearance = YoAppearance.Black();

      AppearanceDefinition expandingPolytopeAppearance = YoAppearance.BlanchedAlmond();

      YoGraphicPosition wYoFramePointViz = new YoGraphicPosition("wYoFramePoint", wYoFramePoint, 0.06, YoAppearance.Black());
      yoGraphicsListRegistry.registerYoGraphic("wYoFramePoint", wYoFramePointViz);

      for (int i = 0; i < maxNumberOfPolytopePoints; i++)
      {
         YoFramePoint polytopeAPoint = new YoFramePoint("polytopeA" + i, ReferenceFrame.getWorldFrame(), registry);
         polytopeAPoint.setToNaN();
         polytopeAPoints.add(polytopeAPoint);
         YoGraphicPosition polytopeAPointViz = new YoGraphicPosition("polytopeA" + i, polytopeAPoint, 0.03, polytopeAAppearance);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeAPoints", polytopeAPointViz);

         YoFramePoint polytopeBPoint = new YoFramePoint("polytopeB" + i, ReferenceFrame.getWorldFrame(), registry);
         polytopeBPoint.setToNaN();
         polytopeBPoints.add(polytopeBPoint);
         YoGraphicPosition polytopeBPointViz = new YoGraphicPosition("polytopeB" + i, polytopeBPoint, 0.03, polytopeBAppearance);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeBPoints", polytopeBPointViz);
      }

      for (int i = 0; i < maxNumberOfPolytopeEdges; i++)
      {
         YoGraphicLineSegment polytopeAEdgeViz = new YoGraphicLineSegment("polytopeAEdge" + i, "", ReferenceFrame.getWorldFrame(), polytopeAAppearance,
               registry);
         polytopeAEdgeViz.setToNaN();
         polytopeAEdgesViz.add(polytopeAEdgeViz);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeAEdges", polytopeAEdgeViz);

         YoGraphicLineSegment polytopeBEdgeViz = new YoGraphicLineSegment("polytopeBEdge" + i, "", ReferenceFrame.getWorldFrame(), polytopeBAppearance,
               registry);
         polytopeBEdgeViz.setToNaN();
         polytopeBEdgesViz.add(polytopeBEdgeViz);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeBEdges", polytopeBEdgeViz);

         YoGraphicLineSegment silhouetteEdgeViz = new YoGraphicLineSegment("silhouetteEdge" + i, "", ReferenceFrame.getWorldFrame(), silhouetteAppearance,
               registry);
         silhouetteEdgeViz.setDrawArrowhead(true);
         silhouetteEdgeViz.setToNaN();
         silhouetteEdgesViz.add(silhouetteEdgeViz);
         yoGraphicsListRegistry.registerYoGraphic("SilhouetteEdges", silhouetteEdgeViz);
      }

      for (int i = 0; i < maxNumberOfExpandingPolygonTriangles; i++)
      {
         YoGraphicTriangle polytopeATriangle = new YoGraphicTriangle("polytopeATriangle" + i + "_", polytopeAAppearance, registry);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeATriangles", polytopeATriangle);
         polytopeATriangle.setToNaN();
         polytopeATriangles.add(polytopeATriangle);

         YoGraphicTriangle polytopeBTriangle = new YoGraphicTriangle("polytopeBTriangle" + i + "_", polytopeBAppearance, registry);
         yoGraphicsListRegistry.registerYoGraphic("PolytopeBTriangles", polytopeBTriangle);
         polytopeBTriangle.setToNaN();
         polytopeBTriangles.add(polytopeBTriangle);

         YoGraphicTriangle expandingPolytopeTriangle = new YoGraphicTriangle("expandingPolytopeTriangle" + i + "_", expandingPolytopeAppearance, registry);
         yoGraphicsListRegistry.registerYoGraphic("ExpandingPolytopeTriangles", expandingPolytopeTriangle);
         expandingPolytopeTriangle.setToNaN();
         expandingPolytopeTriangles.add(expandingPolytopeTriangle);

         yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(polytopeATriangle);
         yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(polytopeBTriangle);
         yoGraphicsListRegistry.registerGraphicsUpdatableToUpdateInAPlaybackListener(expandingPolytopeTriangle);
      }

      for (int i = 0; i < 4; i++)
      {
         simplexPoints[i] = new YoFramePoint("simplexPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition simplexPointViz = new YoGraphicPosition("simplexPoint" + i, simplexPoints[i], 0.03, simplexPointAppearances[i]);
         yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", simplexPointViz);

         correspondingPointsOnPolytopeA[i] = new YoFramePoint("correspondingPointOnPolytopeA" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition correspondingPointOnPolytopeAViz = new YoGraphicPosition("correspondingPointOnPolytopeA" + i, correspondingPointsOnPolytopeA[i],
               0.06, simplexPointAppearances[i]);
         yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", correspondingPointOnPolytopeAViz);

         correspondingPointsOnPolytopeB[i] = new YoFramePoint("correspondingPointOnPolytopeB" + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition correspondingPointOnPolytopeBViz = new YoGraphicPosition("correspondingPointOnPolytopeB" + i, correspondingPointsOnPolytopeB[i],
               0.06, simplexPointAppearances[i]);
         yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", correspondingPointOnPolytopeBViz);

         lambdas[i] = new YoDouble("lambda" + i, registry);

         simplexTriangles[i] = new YoGraphicTriangle("simplexTriangle" + i + "_", simplexTirangleAppearances[i], registry);
         yoGraphicsListRegistry.registerYoGraphic("SimplexTriangles", simplexTriangles[i]);
         simplexTriangles[i].setToNaN();
      }

      closestPointOnExpandingPolytope = new YoFramePoint("closestPointOnExpandingPolytope", ReferenceFrame.getWorldFrame(), registry);
      AppearanceDefinition closestPointAppearance = YoAppearance.Gold();
      closestPointAppearance.setTransparency(0.5);
      YoGraphicPosition closestPointOnExpandingPolytopeViz = new YoGraphicPosition("closestPointOnExpandingPolytope", closestPointOnExpandingPolytope, 0.14,
            closestPointAppearance);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", closestPointOnExpandingPolytopeViz);
      
      closestPointSoFar = new YoFramePoint("closestPointSoFar", ReferenceFrame.getWorldFrame(), registry);
      AppearanceDefinition closestPointSoFarAppearance = YoAppearance.Purple();
      closestPointSoFarAppearance.setTransparency(0.5);
      YoGraphicPosition closestPointSoFarViz = new YoGraphicPosition("closestPointSoFar", closestPointSoFar, 0.08,
            closestPointSoFarAppearance);
      yoGraphicsListRegistry.registerYoGraphic("closestPointSoFar", closestPointSoFarViz);

      closestPointOnA = new YoFramePoint("closestPointOnA", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition closestPointOnAViz = new YoGraphicPosition("closestPointOnAViz", closestPointOnA, 0.12, closestPointAppearance);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", closestPointOnAViz);

      closestPointOnB = new YoFramePoint("closestPointOnB", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition closestPointOnBViz = new YoGraphicPosition("closestPointOnBViz", closestPointOnB, 0.12, closestPointAppearance);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", closestPointOnBViz);

      YoGraphicCoordinateSystem originViz = new YoGraphicCoordinateSystem("origin", "", registry, 1.0);
      yoGraphicsListRegistry.registerYoGraphic("SimplexPoints", originViz);

      scs = new SimulationConstructionSet(new Robot("ExpandingPolytopeVisualizer"));
      scs.setDT(1.0, 1);
      scs.getRootRegistry().addChild(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      scs.updateAndTick();

      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   private void clearAllGraphics()
   {
      clearYoGraphicTriangles(simplexTriangles);
      clearYoGraphicTriangles(polytopeATriangles);
      clearYoGraphicTriangles(polytopeBTriangles);

      closestPointSoFar.setToNaN();
      closestPointOnExpandingPolytope.setToNaN();
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

   private void clearYoFramePoints(ArrayList<YoFramePoint> yoFramePoints)
   {
      for (YoFramePoint yoFramePoint : yoFramePoints)
      {
         yoFramePoint.setToNaN();
      }
   }

   private void clearYoFramePoints(YoFramePoint[] yoFramePoints)
   {
      for (YoFramePoint yoFramePoint : yoFramePoints)
      {
         yoFramePoint.setToNaN();
      }
   }

   private void clearYoGraphicTriangles(ArrayList<YoGraphicTriangle> triangles)
   {
      for (int i = 0; i < triangles.size(); i++)
      {
         triangles.get(i).setToNaN();
      }
   }

   private void clearYoGraphicTriangles(YoGraphicTriangle[] triangles)
   {
      for (int i = 0; i < triangles.length; i++)
      {
         triangles[i].setToNaN();
      }
   }

   private static void drawPolytopeVertices(SupportingVertexHolder polytope, ArrayList<YoFramePoint> polytopeVertcesForViz)
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

   //   @Override
   //   public void addedVertexToSimplex(SimplexPolytope simplex, Point3D vertexOnSimplex, Point3D vertexOnA, Point3D vertexOnB)
   //   {
   //      updateSimplexViz(simplex);
   //      scs.setTime(scs.getTime() + 1.0);
   //      scs.tickAndUpdate();
   //   }
   //
   //   @Override
   //   public void foundClosestPointOnSimplex(SimplexPolytope simplex, Point3D closestPointToOrigin)
   //   {
   //      updateSimplexViz(simplex);
   //      updateLambdas(simplex);
   //
   //      closestPointOnSimplex.set(closestPointToOrigin);
   //      Point3D closestPointOnA = new Point3D();
   //      Point3D closestPointOnB = new Point3D();
   //      simplex.getClosestPointsOnAAndB(closestPointOnA, closestPointOnB);
   //
   //      distanceSimplexToOrigin.set(closestPointToOrigin.distance(new Point3D(0.0, 0.0, 0.0)));
   //      distanceBetweenClosestPoints.set(closestPointOnA.distance(closestPointOnB));
   //
   //      this.closestPointOnA.set(closestPointOnA);
   //      this.closestPointOnB.set(closestPointOnB);
   //
   //      scs.setTime(scs.getTime() + 1.0);
   //      scs.tickAndUpdate();
   //   }

   private void updateTriangleEntriesViz(ExpandingPolytopeEntry triangleEntry)
   {
      Point3D vertexOne = triangleEntry.getVertex(0);
      Point3D vertexTwo = triangleEntry.getVertex(1);
      Point3D vertexThree = triangleEntry.getVertex(2);

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
      }

      if (numberOfPoints == 3)
      {
         this.simplexTriangles[1].setToNaN();
         this.simplexTriangles[2].setToNaN();
         this.simplexTriangles[3].setToNaN();

         Point3D pointOne = simplex.getPoint(0);
         Point3D pointTwo = simplex.getPoint(1);
         Point3D pointThree = simplex.getPoint(2);

         this.simplexTriangles[0].updatePoints(pointOne, pointTwo, pointThree);
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

   @Override
   public void setPolytopes(SimplexPolytope simplex, SupportingVertexHolder polytopeA, SupportingVertexHolder polytopeB, ExpandingPolytopeEntry triangleEntry)
   {
      clearAllGraphics();
      scs.setTime(0.0);

      drawPolytopeVertices(polytopeA, polytopeAPoints);
      drawPolytopeVertices(polytopeB, polytopeBPoints);

      drawPolytopeEdges(polytopeA, polytopeAEdgesViz);
      drawPolytopeEdges(polytopeB, polytopeBEdgesViz);

      updateSimplexViz(simplex);

      updateTriangleEntriesViz(triangleEntry);
      scs.tickAndUpdate();
   }

   @Override
   public void polledEntryToExpand(ExpandingPolytopeEntry triangleEntryToExpand)
   {
      Vector3D closestPointToOrigin = triangleEntryToExpand.getClosestPointToOrigin();
      this.closestPointSoFar.set(closestPointToOrigin);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   @Override
   public void computedSupportingVertices(Point3D supportingVertexA, Point3D supportingVertexB, Vector3D w)
   {
      wYoFramePoint.set(w);
      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   @Override
   public void computedCloseEnough(double vDotW, double lengthSquared, double mu, boolean closeEnough)
   {

   }

   @Override
   public void computedSilhouetteFromW(ExpandingPolytopeEdgeList edgeList)
   {
      for (int i = 0; i < silhouetteEdgesViz.size(); i++)
      {
         silhouetteEdgesViz.get(i).setToNaN();
      }

      for (int i = 0; i < edgeList.getNumberOfEdges(); i++)
      {
         ExpandingPolytopeEdge edge = edgeList.getEdge(i);
         silhouetteEdgesViz.get(i).setStartAndEnd(edge.getStartPoint(), edge.getEndPoint());
      }

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   @Override
   public void addedNewEntryToQueue(ExpandingPolytopeEntry newEntry)
   {

   }

   @Override
   public void createdNewEntry(ExpandingPolytopeEntry newEntry)
   {
      for (int i = 0; i < expandingPolytopeTriangles.size(); i++)
      {
         expandingPolytopeTriangles.get(i).setToNaN();
      }

      ArrayList<ExpandingPolytopeEntry> allTriangles = new ArrayList<>();
      newEntry.getAllConnectedTriangles(allTriangles);

      for (int i = 0; i < allTriangles.size(); i++)
      {
         ExpandingPolytopeEntry triangle = allTriangles.get(i);

         Point3D vertex0 = triangle.getVertex(0);
         Point3D vertex1 = triangle.getVertex(1);
         Point3D vertex2 = triangle.getVertex(2);

         if (!triangle.isObsolete())
            expandingPolytopeTriangles.get(i).updatePoints(vertex0, vertex1, vertex2);
      }

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();
   }

   @Override
   public void expandedPolytope(ExpandingPolytopeEntry firstNewEntry)
   {
      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      firstNewEntry.getAllConnectedTriangles(triangles);

      for (int i = 0; i < triangles.size(); i++)
      {
         ExpandingPolytopeEntry triangle = triangles.get(i);
         if (triangle.isObsolete())
         {
            System.err.println("Triangle is Obsolete!!");
            triangle.checkConsistency();
         }
      }
   }

   @Override
   public void foundMinimumPenetrationVector(Vector3D minimumPenetrationVector, Point3D closestPointOnA, Point3D closestPointOnB)
   {
      this.closestPointOnExpandingPolytope.set(minimumPenetrationVector);
      this.closestPointOnA.set(closestPointOnA);
      this.closestPointOnB.set(closestPointOnB);

      scs.setTime(scs.getTime() + 1.0);
      scs.tickAndUpdate();

   }

}
