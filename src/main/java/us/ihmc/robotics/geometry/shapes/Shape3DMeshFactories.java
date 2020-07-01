package us.ihmc.robotics.geometry.shapes;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class Shape3DMeshFactories
{
   private static final double halfPi = 0.5 * Math.PI;
   private static final double twoPi = 2.0 * Math.PI;

   public static Node toFrameShape3DMesh(FrameShape3DBasics shape3D, Color color)
   {
      FrameShape3DBasics copy = shape3D.copy();
      copy.changeFrame(ReferenceFrame.getWorldFrame());
      return toShape3DMesh(copy, color);
   }

   public static Node toShape3DMesh(Shape3DReadOnly shape3D, Color color)
   {
      if (shape3D instanceof PointShape3DReadOnly)
         return Shape3DMeshFactories.toPointShape3DMesh((PointShape3DReadOnly) shape3D, color);
      if (shape3D instanceof Sphere3DReadOnly)
         return Shape3DMeshFactories.toSphere3DMesh((Sphere3DReadOnly) shape3D, color);
      if (shape3D instanceof Box3DReadOnly)
         return Shape3DMeshFactories.toBox3DMesh((Box3DReadOnly) shape3D, color);
      if (shape3D instanceof Ramp3DReadOnly)
         return Shape3DMeshFactories.toRamp3DMesh((Ramp3DReadOnly) shape3D, color);
      if (shape3D instanceof Ellipsoid3DReadOnly)
         return Shape3DMeshFactories.toEllipsoid3DMesh((Ellipsoid3DReadOnly) shape3D, color);
      if (shape3D instanceof Capsule3DReadOnly)
         return Shape3DMeshFactories.toCapsule3DMesh((Capsule3DReadOnly) shape3D, color);
      if (shape3D instanceof Cylinder3DReadOnly)
         return toCylinder3DMesh((Cylinder3DReadOnly) shape3D, color);
      if (shape3D instanceof ConvexPolytope3DReadOnly)
         return toFace3DsMesh(((ConvexPolytope3DReadOnly) shape3D).getFaces(), color);
      throw new UnsupportedOperationException("Unsupported shape " + shape3D);
   }

   public static Node toPointShape3DMesh(PointShape3DReadOnly pointShape3D, Color color)
   {
      return Shape3DMeshFactories.toPointMesh(pointShape3D, color, 0.01);
   }

   public static Node toSphere3DMesh(Sphere3DReadOnly sphere3D, Color color)
   {
      return Shape3DMeshFactories.toPointMesh(sphere3D.getPosition(), color, sphere3D.getRadius());
   }

   public static Node toBox3DMesh(Box3DReadOnly box3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Cube(box3D.getSizeX(), box3D.getSizeY(), box3D.getSizeZ(), true);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(box3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, box3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toCapsule3DMesh(Capsule3DReadOnly capsule3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Capsule(capsule3D.getLength(), capsule3D.getRadius(), capsule3D.getRadius(), capsule3D.getRadius(), 64, 64);
      mesh = MeshDataHolder.rotate(mesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(capsule3D.getAxis()));
      mesh = MeshDataHolder.translate(mesh, capsule3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toRamp3DMesh(Ramp3DReadOnly ramp3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Wedge(ramp3D.getSizeX(), ramp3D.getSizeY(), ramp3D.getSizeZ());
      mesh = MeshDataHolder.translate(mesh, (float) (0.5 * ramp3D.getSizeX()), 0.0f, 0.0f);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(ramp3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, ramp3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toEllipsoid3DMesh(Ellipsoid3DReadOnly ellipsoid3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Ellipsoid(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ(), 128, 128);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(ellipsoid3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, ellipsoid3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toCylinder3DMesh(Cylinder3DReadOnly cylinder3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Cylinder(cylinder3D.getRadius(), cylinder3D.getLength(), 128);
      mesh = MeshDataHolder.rotate(mesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinder3D.getAxis()));
      mesh = MeshDataHolder.translate(mesh, cylinder3D.getBottomCenter());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static <F extends Face3DReadOnly> Node toFace3DsMesh(List<F> faces)
   {
      Group group = new Group();

      for (F face : faces)
      {
         List<Point3D> cwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         cwFaceVertices.add(cwFaceVertices.get(0));
         cwFaceVertices.add(0, new Point3D(face.getCentroid()));
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         ccwFaceVertices.add(ccwFaceVertices.get(0));
         ccwFaceVertices.add(0, new Point3D(face.getCentroid()));

         MeshView outsideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(ccwFaceVertices)));
         outsideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         outsideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 0.5)));
         group.getChildren().add(outsideFaceNode);

         MeshView insideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(cwFaceVertices)));
         insideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         insideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0).darker()));
         //         group.getChildren().add(insideFaceNode);
      }

      return group;
   }

   public static Node toFace3DsMesh(List<? extends Face3DReadOnly> faces, Color color)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Face3DReadOnly face : faces)
      {
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         meshBuilder.addMesh(MeshDataGenerator.Polygon(ccwFaceVertices));
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toFace3DsNormalMesh(List<? extends Face3DReadOnly> faces)
   {
      Group group = new Group();

      for (int i = 0; i < faces.size(); i++)
      {
         Face3DReadOnly face = faces.get(i);
         double scale = Math.max(0.00003, face.getEdges().stream().mapToDouble(HalfEdge3DReadOnly::length).max().getAsDouble());
         double height = 0.050 * scale;
         double radius = 0.025 * scale;
         AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(face.getNormal());
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         MeshDataHolder cone = MeshDataGenerator.Cone(height, radius, 32);
         cone = MeshDataHolder.rotate(cone, orientation);
         cone = MeshDataHolder.translate(cone, face.getCentroid());
         MeshView normalNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(cone));
         normalNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         normalNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0)));
         group.getChildren().add(normalNode);
      }

      return group;
   }

   public static Node toPointMesh(Tuple3DReadOnly position, Color color, double size)
   {
      MeshDataHolder sphereMeshData = MeshDataHolder.translate(MeshDataGenerator.Sphere(size, 64, 64), position);
      MeshView node = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(sphereMeshData));
      node.setMaterial(new PhongMaterial(color));
      return node;
   }

   public static Node toHalfEdge3DsMesh(List<? extends HalfEdge3DReadOnly> edges, Color color, double width)
   {
      Group group = new Group();

      if (edges == null)
         return group;

      for (HalfEdge3DReadOnly edge : edges)
      {
         MeshDataHolder line = MeshDataGenerator.Line(edge.getOrigin(), edge.getDestination(), width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + edges.indexOf(edge) + ": " + edge.toString()));
         group.getChildren().add(edgeNode);
      }

      return group;
   }

   public static Node toMultilineMesh(List<? extends Point3DReadOnly> multiline, boolean close, Color color, double width)
   {
      Group group = new Group();

      if (multiline == null)
         return group;

      if (multiline.size() < 2)
         return group;

      for (int i = 1; i < multiline.size(); i++)
      {
         Point3DReadOnly start = multiline.get(i - 1);
         Point3DReadOnly end = multiline.get(i);
         MeshDataHolder line = MeshDataGenerator.Line(start, end, width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         group.getChildren().add(edgeNode);
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED,
                                  e -> System.out.println("start: " + EuclidCoreIOTools.getTuple3DString(start) + ", end: "
                                        + EuclidCoreIOTools.getTuple3DString(end)));
      }

      if (close)
      {
         Point3DReadOnly start = multiline.get(multiline.size() - 1);
         Point3DReadOnly end = multiline.get(0);
         MeshDataHolder line = MeshDataGenerator.Line(start, end, width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         group.getChildren().add(edgeNode);
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED,
                                  e -> System.out.println("start: " + EuclidCoreIOTools.getTuple3DString(start) + ", end: "
                                        + EuclidCoreIOTools.getTuple3DString(end)));
      }

      return group;
   }

   public enum UVMeshType
   {
      HULL, SUPPORT_VERTICES, SUPPORT_DIRECTIONS
   }

   public static Node toUVMesh(SupportingVertexHolder supportingVertexHolder, Color color)
   {
      return toUVMesh(supportingVertexHolder, color, 256, 256, UVMeshType.HULL);
   }

   public static Node toUVMesh(SupportingVertexHolder supportingVertexHolder, Color color, int latitudeN, int longitudeN, UVMeshType meshType)
   {
      Vector3D32[] supportDirections = new Vector3D32[(latitudeN - 1) * longitudeN + 2];
      Point3D32 points[] = new Point3D32[(latitudeN - 1) * longitudeN + 2];
      TexCoord2f textPoints[] = new TexCoord2f[(latitudeN - 1) * longitudeN + 2];

      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         float longitudeAngle = (float) (twoPi * ((float) longitudeIndex / (float) longitudeN));
         float cosLongitude = (float) Math.cos(longitudeAngle);
         float sinLongitude = (float) Math.sin(longitudeAngle);

         for (int latitudeIndex = 1; latitudeIndex < latitudeN; latitudeIndex++)
         {
            float latitudeAngle = (float) (-halfPi + Math.PI * ((float) latitudeIndex / (float) latitudeN));
            float cosLatitude = (float) Math.cos(latitudeAngle);
            float sinLatitude = (float) Math.sin(latitudeAngle);

            int currentIndex = (latitudeIndex - 1) * longitudeN + longitudeIndex;
            supportDirections[currentIndex] = new Vector3D32(cosLongitude * cosLatitude, sinLongitude * cosLatitude, sinLatitude);
            points[currentIndex] = new Point3D32(supportingVertexHolder.getSupportingVertex(supportDirections[currentIndex]));

            float textureX = (float) (longitudeAngle / twoPi);
            float textureY = (float) (0.5 * sinLatitude + 0.5);
            textPoints[currentIndex] = new TexCoord2f(textureX, textureY);
         }
      }

      // South pole
      int southPoleIndex = (latitudeN - 1) * longitudeN;
      supportDirections[southPoleIndex] = new Vector3D32(0.0f, 0.0f, -1.0f);
      points[southPoleIndex] = new Point3D32(supportingVertexHolder.getSupportingVertex(supportDirections[southPoleIndex]));
      textPoints[southPoleIndex] = new TexCoord2f(0.5f, 0.0f);

      // North pole
      int northPoleIndex = (latitudeN - 1) * longitudeN + 1;
      supportDirections[northPoleIndex] = new Vector3D32(0.0f, 0.0f, 1.0f);
      points[northPoleIndex] = new Point3D32(supportingVertexHolder.getSupportingVertex(supportDirections[northPoleIndex]));
      textPoints[northPoleIndex] = new TexCoord2f(1.0f, 1.0f);

      int numberOfTriangles = 2 * (latitudeN - 1) * longitudeN + 2 * longitudeN;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      // Mid-latitude faces
      for (int latitudeIndex = 0; latitudeIndex < latitudeN - 2; latitudeIndex++)
      {
         for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
         {
            int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
            int nextLatitudeIndex = latitudeIndex + 1;

            // Lower triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + longitudeIndex;
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
            // Upper triangles
            triangleIndices[index++] = latitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + nextLongitudeIndex;
            triangleIndices[index++] = nextLatitudeIndex * longitudeN + longitudeIndex;
         }
      }

      // South pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = southPoleIndex;
         triangleIndices[index++] = nextLongitudeIndex;
         triangleIndices[index++] = longitudeIndex;
      }

      // North pole faces
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         int nextLongitudeIndex = (longitudeIndex + 1) % longitudeN;
         triangleIndices[index++] = northPoleIndex;
         triangleIndices[index++] = (latitudeN - 2) * longitudeN + longitudeIndex;
         triangleIndices[index++] = (latitudeN - 2) * longitudeN + nextLongitudeIndex;
      }

      if (meshType == UVMeshType.HULL)
      {
         MeshDataHolder rawMesh = new MeshDataHolder(points, textPoints, triangleIndices, supportDirections);
         TriangleMesh jfxMesh = JavaFXMeshDataInterpreter.interpretMeshData(rawMesh, true);
         MeshView meshView = new MeshView(jfxMesh);
         meshView.setMaterial(new PhongMaterial(color));
         return meshView;
      }
      else if (meshType == UVMeshType.SUPPORT_VERTICES)
      {
         JavaFXMeshBuilder builder = new JavaFXMeshBuilder();
         for (int i = 0; i < points.length; i++)
         {
            Point3D32 point = points[i];
            builder.addTetrahedron(0.005f, point);
//            builder.addMesh(MeshDataGenerator.Sphere(0.0005f, 6, 6), point);
         }
         MeshView meshView = new MeshView(builder.generateMesh());
         meshView.setMaterial(new PhongMaterial(color));
         return meshView;
      }
      else if (meshType == UVMeshType.SUPPORT_DIRECTIONS)
      {
         JavaFXMeshBuilder builder = new JavaFXMeshBuilder();
         for (int i = 0; i < points.length; i++)
         {
            Point3D32 point = points[i];
            Vector3D32 direction = supportDirections[i];
            MeshDataHolder directionMesh = MeshDataGenerator.Cylinder(0.005, 0.1, 3);
            builder.addMesh(directionMesh, point, EuclidGeometryTools.axisAngleFromZUpToVector3D(direction));
         }
         MeshView meshView = new MeshView(builder.generateMesh());
         meshView.setMaterial(new PhongMaterial(color));
         return meshView;
      }
      else
      {
         throw new IllegalStateException("Unexpected mesh type: " + meshType);
      }
   }

   public static Color nextColor(Random random)
   {
      return Color.hsb(EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0), 0.9, 0.9);
   }
}
