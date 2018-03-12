package us.ihmc.scsVisualizers.screwTools;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collector;
import java.util.stream.Collectors;

import javafx.application.Application;
import javafx.scene.AmbientLight;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.Ellipsoid3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotics.geometry.shapes.FrameEllipsoid3d;
import us.ihmc.robotics.screwTheory.Twist;

public class TwistVisualizer extends Application
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame ellipsoidFrame;
   private final RigidBodyTransform ellipsoidTransform = new RigidBodyTransform();
   private final FrameEllipsoid3d frameEllipsoid;
   private final Twist ellipsoidCenterTwist = new Twist();

   public TwistVisualizer()
   {
      ellipsoidTransform.setTranslation(0.3, 0.3, 0.4);
      ellipsoidTransform.setRotationYawPitchRoll(0.5, 0.5 * Math.PI-0.5, 0.0);
      ellipsoidFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("ellipsoidFrame", worldFrame, ellipsoidTransform);
      frameEllipsoid = new FrameEllipsoid3d(ellipsoidFrame, 0.2, 0.2, 0.3);
      ellipsoidCenterTwist.setToZero(ellipsoidFrame, worldFrame, ellipsoidFrame);
      ellipsoidCenterTwist.setLinearPart(0.0, 0.0, 0.0);
      ellipsoidCenterTwist.setAngularPart(-0.3, -0.3, 0.9);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(800, 600);
      view3dFactory.addCameraController();
      view3dFactory.addPointLight(0.0, 0.0, 1.0);
      view3dFactory.addNodeToView(new AmbientLight(Color.LIGHTGRAY));
      view3dFactory.addWorldCoordinateSystem(0.3);

      Ellipsoid3D ellipsoid = frameEllipsoid.getEllipsoid3d();
      MeshDataHolder ellipsoidMesh = MeshDataGenerator.Ellipsoid(ellipsoid.getRadiusX(), ellipsoid.getRadiusY(), ellipsoid.getRadiusZ(), 32, 32);
      ellipsoidMesh.applyTransform(ellipsoidTransform);
      MeshView ellipsoidMeshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(ellipsoidMesh));
//      ellipsoidMeshView.setDrawMode(DrawMode.LINE);
      ellipsoidMeshView.setMaterial(new PhongMaterial(Color.DARKGREEN));
      view3dFactory.addNodeToView(ellipsoidMeshView);

      List<Twist> surfaceTwists = computeSurfaceTwists(10, 10);
      List<Node> linearVelocityVectors = createLinearVelocityVectors(0.6, surfaceTwists, Color.DARKGRAY);
      view3dFactory.addNodesToView(linearVelocityVectors);
      view3dFactory.addNodeToView(createAngularVelocityVector(0.7, Color.RED, ellipsoidCenterTwist));

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private List<Twist> computeSurfaceTwists(int latitudeN, int longitudeN)
   {
      List<Twist> twistsOnSurface = new ArrayList<>();

      Ellipsoid3D ellipsoid3d = frameEllipsoid.getEllipsoid3d();
      double xRadius = ellipsoid3d.getRadiusX();
      double yRadius = ellipsoid3d.getRadiusY();
      double zRadius = ellipsoid3d.getRadiusZ();
      
      
      for (int longitudeIndex = 0; longitudeIndex < longitudeN; longitudeIndex++)
      {
         double longitudeAngle = 2.0 * Math.PI * (longitudeIndex / (double) longitudeN);
         double cosLongitude = Math.cos(longitudeAngle);
         double sinLongitude = Math.sin(longitudeAngle);

         for (int latitudeIndex = 1; latitudeIndex < latitudeN; latitudeIndex++)
         {
            double latitudeAngle = (-0.5 * Math.PI + Math.PI * (latitudeIndex / (double) latitudeN));
            double cosLatitude = Math.cos(latitudeAngle);
            double sinLatitude = Math.sin(latitudeAngle);

            double surfaceX = xRadius * cosLongitude * cosLatitude;
            double surfaceY = yRadius * sinLongitude * cosLatitude;
            double surfaceZ = zRadius * sinLatitude;
            RigidBodyTransform transformFromSurfaceToCenter = new RigidBodyTransform();
            transformFromSurfaceToCenter.setTranslation(surfaceX, surfaceY, surfaceZ);
            String frameName = "surfaceFrameLong" + longitudeIndex + "Latitude" + latitudeIndex;
            ReferenceFrame frameOnSurface = ReferenceFrame.constructFrameWithUnchangingTransformToParent(frameName, ellipsoidFrame, transformFromSurfaceToCenter);
            Twist surfaceTwist = new Twist(ellipsoidCenterTwist);
            surfaceTwist.changeFrame(frameOnSurface);
            twistsOnSurface.add(surfaceTwist);
         }
      }

      return twistsOnSurface;
   }

   private List<Node> createLinearVelocityVectors(double scale, List<Twist> twists, Color color)
   {
      return twists.stream().map(twist -> createLinearVelocityVector(scale, color, twist)).collect(Collectors.toList());
   }

   public MeshView createLinearVelocityVector(double scale, Color color, Twist twist)
   {
      FrameVector3D linearVelocity = new FrameVector3D();
      twist.getLinearPart(linearVelocity);
      return createArrow(scale, color, linearVelocity);
   }

   public MeshView createAngularVelocityVector(double scale, Color color, Twist twist)
   {
      FrameVector3D angularVelocity = new FrameVector3D();
      twist.getAngularPart(angularVelocity);
      return createArrow(scale, color, angularVelocity);
   }

   @SuppressWarnings("deprecation")
   public MeshView createArrow(double scale, Color color, FrameVector3D vector)
   {
      double length = scale * vector.length();
      MeshDataBuilder meshDataBuilder = new MeshDataBuilder();
      AxisAngle axisAngleToVector = EuclidGeometryTools.axisAngleFromZUpToVector3D(vector);
      meshDataBuilder.addCylinder(length, 0.02 * length, new Point3D());
      meshDataBuilder.addCone(0.10 * length, 0.05 * length, new Point3D(0.0, 0.0, length));
      MeshDataHolder meshDataHolder = meshDataBuilder.generateMeshDataHolder();
      meshDataHolder = MeshDataHolder.rotate(meshDataHolder, axisAngleToVector);
      meshDataHolder.applyTransform(vector.getReferenceFrame().getTransformToWorldFrame());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(meshDataHolder));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
