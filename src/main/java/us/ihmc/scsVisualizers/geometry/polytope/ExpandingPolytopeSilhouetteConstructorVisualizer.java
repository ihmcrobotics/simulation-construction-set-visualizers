package us.ihmc.scsVisualizers.geometry.polytope;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.geometry.polytope.ConvexPolytopeFromExpandingPolytopeEntryGenerator;
import us.ihmc.geometry.polytope.ExpandingPolytopeEdge;
import us.ihmc.geometry.polytope.ExpandingPolytopeEdgeList;
import us.ihmc.geometry.polytope.ExpandingPolytopeEntry;
import us.ihmc.geometry.polytope.ExpandingPolytopeEntryFromSimpleMeshGenerator;
import us.ihmc.geometry.polytope.ExpandingPolytopeSilhouetteConstructor;
import us.ihmc.geometry.polytope.IcoSphereCreator;
import us.ihmc.geometry.polytope.PolytopeVertex;
import us.ihmc.geometry.polytope.SimpleTriangleMesh;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicLineSegment;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * Visualizer for ExpandingPolytopeSilhouetteConstructor. Used for debugging the silhouette part of the Expanding Polytope Algorithm.
 *
 */
public class ExpandingPolytopeSilhouetteConstructorVisualizer
{

   public ExpandingPolytopeSilhouetteConstructorVisualizer()
   {
      Robot robot = new Robot(getClass().getSimpleName() + "Robot");

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      YoGraphicLineSegment[] silhouetteLineSegmentsViz = new YoGraphicLineSegment[120];
      YoGraphicLineSegment[] polytopeEdgesViz = new YoGraphicLineSegment[120];
      YoGraphicPosition[] triangleClosestPoints = new YoGraphicPosition[80];

      for (int i = 0; i < silhouetteLineSegmentsViz.length; i++)
      {
         YoGraphicLineSegment lineSegmentViz = new YoGraphicLineSegment("silhouetteEdge" + i, "", ReferenceFrame.getWorldFrame(), YoAppearance.Purple(),
               registry);
         lineSegmentViz.setDrawArrowhead(true);
         lineSegmentViz.setLineRadiusWhenOneMeterLong(0.025);

         yoGraphicsListRegistry.registerYoGraphic("SilhouetteEdges", lineSegmentViz);
         lineSegmentViz.setToNaN();
         silhouetteLineSegmentsViz[i] = lineSegmentViz;
      }

      for (int i = 0; i < triangleClosestPoints.length; i++)
      {
         YoGraphicPosition triangleClosestPoint = new YoGraphicPosition("triangleClosestPoint" + i, "", registry, 0.005, YoAppearance.Gold());
         yoGraphicsListRegistry.registerYoGraphic("triangleClosestPoints", triangleClosestPoint);
         triangleClosestPoint.setPositionToNaN();
         triangleClosestPoints[i] = triangleClosestPoint;
      }

      for (int i = 0; i < polytopeEdgesViz.length; i++)
      {
         YoGraphicLineSegment lineSegmentViz = new YoGraphicLineSegment("polytopeEdge" + i, "", ReferenceFrame.getWorldFrame(), YoAppearance.Black(), registry);
         lineSegmentViz.setDrawArrowhead(false);

         yoGraphicsListRegistry.registerYoGraphic("PolytopeEdges", lineSegmentViz);
         lineSegmentViz.setToNaN();
         polytopeEdgesViz[i] = lineSegmentViz;
      }

      YoFramePoint3D wPosition = new YoFramePoint3D("wPosition", ReferenceFrame.getWorldFrame(), registry);
      YoGraphicPosition wPositionViz = new YoGraphicPosition("wPosition", wPosition, 0.02, YoAppearance.Gold());
      yoGraphicsListRegistry.registerYoGraphic("WPosition", wPositionViz);

      YoInteger numberOfSilhouetteEdges = new YoInteger("numberOfSilhouetteEdges", registry);

      robot.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      robot.addYoVariableRegistry(registry);

      IcoSphereCreator creator = new IcoSphereCreator();
      int recursionLevel = 0;
      SimpleTriangleMesh icoSphere = creator.createIcoSphere(recursionLevel);

      ExpandingPolytopeEntryFromSimpleMeshGenerator generator = new ExpandingPolytopeEntryFromSimpleMeshGenerator();
      ExpandingPolytopeEntry expandingPolytope = generator.generateExpandingPolytope(icoSphere);

      ConvexPolytopeFromExpandingPolytopeEntryGenerator generatorTwo = new ConvexPolytopeFromExpandingPolytopeEntryGenerator();
      ConvexPolytope convexPolytope = generatorTwo.generateConvexPolytope(expandingPolytope);
      drawPolytopeEdges(convexPolytope, polytopeEdgesViz);

      ArrayList<ExpandingPolytopeEntry> triangles = new ArrayList<>();
      expandingPolytope.getAllConnectedTriangles(triangles);

      for (int i = 0; i < triangles.size(); i++)
      {
         ExpandingPolytopeEntry triangle = triangles.get(i);
         Vector3D closestPointToOrigin = triangle.getClosestPointToOrigin();
         triangleClosestPoints[i].setPosition(closestPointToOrigin);
      }

      YoFramePoint3D wFramePoint = new YoFramePoint3D("wFramePoint", ReferenceFrame.getWorldFrame(), registry);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      Graphics3DObject coordinateSystem = new Graphics3DObject();
      coordinateSystem.addCoordinateSystem(1.0);
      scs.addStaticLinkGraphics(coordinateSystem);
      scs.setGroundVisible(false);
      scs.setDT(1.0, 1);
      scs.startOnAThread();

      for (double z = 1.0; z < 5.0; z = z + 0.01)
      {
         wFramePoint.set(1.0, 0.0, z);

         visualizeForW(silhouetteLineSegmentsViz, wPosition, numberOfSilhouetteEdges, triangles, new Vector3D(wFramePoint));
         scs.setTime(scs.getTime() + 1.0);
         scs.tickAndUpdate();
      }

      scs.cropBuffer();
   }

   private static void visualizeForW(YoGraphicLineSegment[] silhouetteLineSegmentsViz, YoFramePoint3D wPosition, YoInteger numberOfSilhouetteEdges,
         ArrayList<ExpandingPolytopeEntry> triangles, Vector3D w)
   {
      for (YoGraphicLineSegment silhouetteLineSegmentViz : silhouetteLineSegmentsViz)
      {
         silhouetteLineSegmentViz.setToNaN();
      }

      wPosition.set(w);

      ArrayList<ExpandingPolytopeEntry> trianglesVisibleFromW = new ArrayList<>();
      for (int i = 0; i < triangles.size(); i++)
      {
         ExpandingPolytopeEntry triangle = triangles.get(i);
         Vector3D closestPointToOrigin = triangle.getClosestPointToOrigin();
         if (!ExpandingPolytopeSilhouetteConstructor.isNotVisibleFromW(closestPointToOrigin, w))
         {
            trianglesVisibleFromW.add(triangle);
         }
      }

      int numberOfEdges = -1;
      for (ExpandingPolytopeEntry triangle : trianglesVisibleFromW)
      {
         clearObsolete(triangles);

         ExpandingPolytopeEdgeList edgeListToPack = new ExpandingPolytopeEdgeList();
         ExpandingPolytopeSilhouetteConstructor.computeSilhouetteFromW(triangle, w, edgeListToPack);

         if (numberOfEdges == -1)
         {
            numberOfEdges = edgeListToPack.getNumberOfEdges();
            numberOfSilhouetteEdges.set(numberOfEdges);
         }
         else
         {
            if (edgeListToPack.getNumberOfEdges() != numberOfEdges)
            {
               throw new RuntimeException("Getting different number of edges from different starting visible triangles!?");
            }
         }

         //         System.out.print("\n\nEdgeList:\n");

         for (int i = 0; i < numberOfEdges; i++)
         {
            ExpandingPolytopeEdge edge = edgeListToPack.getEdge(i);
            ExpandingPolytopeEntry entry = edge.getEntry();
            int edgeIndex = edge.getEdgeIndex();
            Point3D vertexOne = entry.getVertex(edgeIndex);
            Point3D vertexTwo = entry.getVertex((edgeIndex + 1) % 3);
            //            System.out.println(vertexOne + " -- " + vertexTwo);

            silhouetteLineSegmentsViz[i].setStartAndEnd(vertexOne, vertexTwo);
         }
      }
   }

   private static void clearObsolete(ArrayList<ExpandingPolytopeEntry> triangles)
   {
      for (ExpandingPolytopeEntry triangle : triangles)
      {
         triangle.clearObsolete();
      }
   }

   private static void drawPolytopeEdges(ConvexPolytope polytope, YoGraphicLineSegment[] polytopeEdgesViz)
   {
      int numberOfEdges = polytope.getNumberOfEdges();
      ArrayList<PolytopeVertex[]> edges = polytope.getEdges();

      for (int i = 0; i < numberOfEdges; i++)
      {
         PolytopeVertex[] edge = edges.get(i);
         YoGraphicLineSegment polytopeEdgeViz = polytopeEdgesViz[i];
         polytopeEdgeViz.setStartAndEnd(edge[0].getPosition(), edge[1].getPosition());
      }
   }

   public static void main(String[] args)
   {
      new ExpandingPolytopeSilhouetteConstructorVisualizer();
   }

}
