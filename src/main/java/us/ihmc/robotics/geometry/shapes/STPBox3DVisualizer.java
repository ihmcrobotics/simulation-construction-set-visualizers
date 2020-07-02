package us.ihmc.robotics.geometry.shapes;

import java.util.List;
import java.util.Random;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.AmbientLight;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotics.geometry.shapes.Shape3DMeshFactories.UVMeshType;

public class STPBox3DVisualizer extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(600, 400);
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.001, 100.0, true);
      cameraController.setMinLatitude(Double.NEGATIVE_INFINITY);
      cameraController.setMaxLatitude(Double.POSITIVE_INFINITY);
      view3dFactory.addWorldCoordinateSystem(0.1);
      view3dFactory.addNodeToView(new AmbientLight(Color.GRAY));
      view3dFactory.addPointLight(-10.0, 0.0, -1.0, Color.WHEAT);
      //      PointLight light = new PointLight(Color.WHEAT);
      //      light.getTransforms().addAll(view3dFactory.getScene().getCamera().getTransforms());
      //      view3dFactory.addNodeToView(light);

      Random random = new Random(34106);
      for (int i = 0; i < 10; i++)
      {
         Box3D box3D = EuclidShapeRandomTools.nextBox3D(random);
         box3D.getSize().set(0.25, 0.5, 0.75);
//         box3D.getPose().setToZero();
         box3D.getPose().getTranslation().set(EuclidCoreRandomTools.nextPoint3D(random, 2.0));
         STPBox3D stpBox = new STPBox3D(box3D);
         stpBox.setMargins(0.05, 0.15);
         view3dFactory.addNodeToView(Shape3DMeshFactories.toBox3DMesh(box3D, Color.DARKCYAN));

         view3dFactory.addNodeToView(STPShape3DMeshBuilder.toSTPBox3DMesh(stpBox,
                                                                                        Color.CORNFLOWERBLUE,
                                                                                        Color.BLUEVIOLET,
                                                                                        Color.DARKORANGE,
                                                                                        true));
         //      view3dFactory.addNodeToView(Shape3DMeshFactories.toShape3DMesh(box3D.asConvexPolytope(), Color.PINK.deriveColor(0.0, 1.0, 1.0, 0.2)));
         //      view3dFactory.addNodeToView(Shape3DMeshFactories.toFace3DsNormalMesh(box3D.asConvexPolytope().getFaces()));
         //      view3dFactory.addNodeToView(Shape3DMeshFactories.toHalfEdge3DsMesh(box3D.asConvexPolytope().getHalfEdges(), Color.BLACK, 0.01));
         int resolution = 150;
         //         view3dFactory.addNodeToView(Shape3DMeshFactories.toUVMesh(stpBox,
         //                                                                   Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2),
         //                                                                   resolution,
         //                                                                   resolution, UVMeshType.HULL));
         Node supportVerticesMesh = Shape3DMeshFactories.toUVMesh(stpBox,
                                                                  Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2),
                                                                  resolution,
                                                                  resolution,
                                                                  UVMeshType.SUPPORT_VERTICES);
         supportVerticesMesh.setMouseTransparent(true);
         view3dFactory.addNodeToView(supportVerticesMesh);
      }

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.setOnCloseRequest(event -> stop());
      primaryStage.show();
   }

   @Override
   public void stop()
   {
      Platform.exit();
   }

   public static Node generateTetrahedronsMesh(List<? extends Point3DReadOnly> points, Color color, double size)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Point3DReadOnly point : points)
      {
         meshBuilder.addTetrahedron(size, point);
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}