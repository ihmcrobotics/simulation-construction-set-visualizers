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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.robotics.geometry.shapes.Shape3DMeshFactories.UVMeshType;
import us.ihmc.commons.robotics.robotSide.RobotSide;

public class STPConvexPolytope3DVisualizer extends Application
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

      Random random = new Random(524);

      STPConvexPolytope3D stpPolytope = new STPConvexPolytope3D(atlasFootCollision());
//      stpPolytope.setMargins(0.08, 0.10);
      view3dFactory.addNodeToView(Shape3DMeshFactories.toFace3DsMesh(stpPolytope.getFaces(), Color.DARKCYAN));
      view3dFactory.addNodeToView(STPShape3DMeshBuilder.toSTPConvexPolytope3DMesh(stpPolytope, Color.CORNFLOWERBLUE, Color.BLUEVIOLET, Color.DARKORANGE, false));
      int resolution = 150;
      //      view3dFactory.addNodeToView(Shape3DMeshFactories.toUVMesh(stpPolytope, Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2), resolution, resolution, UVMeshType.HULL));
      view3dFactory.addNodeToView(Shape3DMeshFactories.toUVMesh(stpPolytope,
                                                                Color.DARKRED.deriveColor(0.0, 1.0, 1.0, 0.2),
                                                                resolution,
                                                                resolution,
                                                                UVMeshType.SUPPORT_VERTICES));

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.setOnCloseRequest(event -> stop());
      primaryStage.show();
   }

   public static STPConvexPolytope3D atlasFootCollision()
   {
      STPConvexPolytope3D footShape = new STPConvexPolytope3D(1.0e-5);

      for (RobotSide footSide : RobotSide.values)
      {
         for (double z : new double[] {0.077, 0.03})
         {
            // Heel
            footShape.addVertex(new Point3D(-0.08, footSide.negateIfRightSide(0.065), z));
            // Start toe taper
            footShape.addVertex(new Point3D(+0.12, footSide.negateIfRightSide(0.065), z));
         }
         // Toe
         footShape.addVertex(new Point3D(+0.17, footSide.negateIfRightSide(0.035), 0.077));
      }

      footShape.setMargins(1.0e-5, 10e-3);

      return footShape;
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