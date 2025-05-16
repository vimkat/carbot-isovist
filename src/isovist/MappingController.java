package isovist;

import basics.math.Geom;
import basics.points.PointCloudCreator2D;
import basics.points.PointList2D;
import java.util.ArrayList;
import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.RobotGeomUtil;
import robotinterface.Time;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.LidarSlamWorldModelPoint;
import robotinterface.lss.LidarSubsystem;
import robotinterface.lss.LidarSubsystemListenerSlam;
import robotinterface.lss.ObservedLidarPointSlam;
import robotinterface.mss.AsyncMotionMessage;
import robotinterface.mss.AsyncMotionMessageBundle;
import robotinterface.mss.MotionSubsystem;
import robotinterface.mss.MotionSubsystemListener;
import robotlib.driver.Driver;
import robotlib.driver.RegulatedAheadDriver;
import robotlib.nav.Pos2PosRouting;
import robotlib.nav.grid.Grid_Astar;
import robotlib.nav.grid.Pos2PosRoutingGrid;
import robotlib.navtraj.NavTrajPlanning;
import robotlib.navtraj.NavTrajSplitPlanning;
import robotlib.navtraj.RouteTraj;
import robotlib.traj.TrajectoryPlanner;
import robotlib.traj.longrange.LongrangePlanner;
import robotlib.traj.longrange.LongrangeProfile;
import robotlib.traj.seq.Maneuver;
import robotlib.traj.seq.TrajectorySequence;
import robotlib.worldmodel.ObstacleContainer;
import robotlib.worldmodel.ObstaclePoint;

import java.util.ArrayList;

import basics.points.PointCloudCreator2D;
import basics.points.PointList2D;
import basics.points.Point;
import basics.points.container.GridPointCloud2D;
import basics.points.container.ArrayPointList;
import robotinterface.Robot;
import robotinterface.RobotController;
import robotinterface.Time;
import robotinterface.debug.DebugPainterOverlay;
import robotinterface.lss.LidarPackageSlam;
import robotinterface.lss.LidarSubsystem;
import robotinterface.lss.LidarSubsystemListenerSlam;
import robotinterface.mss.AsyncMotionMessageBundle;
import robotinterface.mss.MotionSubsystemListener;
import robotlib.driver.Driver;
import robotlib.worldmodel.ObstacleContainer;
import robotlib.worldmodel.ObstaclePoint;


public class MappingController
  extends RobotController
  implements
    MotionSubsystemListener,
    // LidarSubsystemListenerRaw,
    LidarSubsystemListenerSlam {

	double posX, posY, posAng;
	double us;
	boolean tactile;

	ObstacleContainer obstaclesLidar;
	GridPointCloud2D cloud;

  public MappingController() {
    Robot.motionSubsystem.registerMotionListener(this);
    // Receive MMS messages
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "No Lidar Subsystem available - I cannot see anything!"
      );
      return;
    }
    try {
      Robot.lidarSubsystem.setTiming(LidarSubsystem.EQUIDISTANT, 500);
      // Robot.lidarSubsystem.registerLidarListenerRaw(this); // Receive raw Lidar points
    } catch (UnsupportedOperationException e) {
      Robot.debugOut.println("Lidar Subsystem does not provide raw points");
    }

    try {
      Robot.lidarSubsystem.registerLidarListenerSlam(this); // Receive corr. Lidar points
      Robot.lidarSubsystem.setMSSCorrection(true);
      // Lidar-SLAM correct position
    } catch (UnsupportedOperationException e) {
      Robot.debugOut.println(
        "Lidar Subsystem does not provide SLAM-corrected points"
      );
    }

    // Hindernis-Karte einrichten
    obstaclesLidar = new ObstacleContainer(
        PointCloudCreator2D.TYPE_GRID,
        10.0d,
        ObstacleContainer.ADD_MODE_INSERT,
        3.0d); // FUSION_MAX_LIDAR_DIST

		// HACK: This doesn't work in real life
		double[][] obstacles = Robot.lidarSubsystem.getAllObstacles();
		cloud = new GridPointCloud2D(10, obstacles, Point.class);
		paintObstacles(cloud.getAll2D(), "All Obstacle Points");
  }

  private void paintObstacles(double[][] obstacles, String overlayStr) {
    DebugPainterOverlay ovl = Robot.debugPainter.getOverlay(overlayStr);
    ovl.clear();
    for (int i = 0; i < obstacles.length; i++)
      ovl.fillCircle(obstacles[i][0], obstacles[i][1], 5, 200, 0, 0, 255);
    ovl.paint();
  }

  public String getDescription() {
    return "Just a demo controller (Lidar)";
  }

  public boolean requiresConfiguration() {
    return false;
  }

  // This controller needs config
  public void configure(String params) throws IllegalArgumentException {
    // ...
    // Accept a configuration and store it
  }

  public void run() throws Exception {
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "This Robot Controller requires a Lidar Subsystem"
      );
      return;
    }
    try {
      Robot.lidarSubsystem.resetWorldModel();
    } catch (UnsupportedOperationException e) {
      Robot.debugOut.println("Lidar Subsystem does not use a World Model");
    }
    Robot.lidarSubsystem.startup();
    Robot.motionSubsystem.sendCommand("stoprule T,U50");
    // Demo init
    Robot.motionSubsystem.sendCommand("fore 400");
    // Do something reasonable
    while (isRunning()) {
      // ...
      Time.sleep(100);
      // Do not consume all the CPU power
      // ...
    }
  }

  public void pause() throws Exception {
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "This Robot Controller requires a Lidar Subsystem"
      );
      return;
    }
    Robot.lidarSubsystem.shutdown();
    Robot.motionSubsystem.sendCommand("stop");
  }

  public void stop() throws Exception {
    if (Robot.lidarSubsystem == null) {
      Robot.debugOut.println(
        "This Robot Controller requires a Lidar Subsystem"
      );
      return;
    }
    Robot.lidarSubsystem.shutdown();
    Robot.motionSubsystem.sendCommand("stop");
  }

  // ...
  // Stop everything (e.g. motors)
  // and clear state
  public void mssResponse(ArrayList<String> messages, int responseType)
    throws Exception {
    if (
      MotionSubsystemListener.isFailureResponse(responseType)
    ) Robot.debugOut.println("Failure response " + messages.get(0));
  }

  public void mssAsyncMessages(
    ArrayList<String> messages,
    AsyncMotionMessageBundle bundle
  ) throws Exception {
    if (bundle.containsPos()) {
      posX = bundle.getDouble(AsyncMotionMessage.X);
      // Current pos x
      posY = bundle.getDouble(AsyncMotionMessage.Y);
      // Current pos y
      posAng = bundle.getDouble(AsyncMotionMessage.ANG);
      // Current angle
    }
    if (bundle.containsType(AsyncMotionMessage.TACTIL)) tactile =
      bundle.getBoolean(AsyncMotionMessage.TACTIL); // Taktil value
    if (bundle.containsType(AsyncMotionMessage.US)) us = bundle.getDouble(
      AsyncMotionMessage.US
    );
    // US distance
    if (bundle.containsType(AsyncMotionMessage.COLLISION_TACTIL)) {
      // ...
      // Taktil collision detected!
    }
    if (bundle.containsType(AsyncMotionMessage.COLLISION_US)) {
      // ...
      // US collision detected!
    }
  }

  // public void observedLidarPointsRaw(LidarPackageRaw lidarPackageRaw)
  //   throws Exception {
  //   // ...
  //   // Process Raw Scan
  // }
	

  public void observedLidarPointsSlam(LidarPackageSlam lidarPackageSlam)
    throws Exception {

		// Idk if this is right
		posX = lidarPackageSlam.observationPosX;
		posY = lidarPackageSlam.observationPosY;

		ArrayList<ObservedLidarPointSlam> points = lidarPackageSlam.observedPoints;

		// Paint seen lidar points
    DebugPainterOverlay ovl = Robot.debugPainter.getOverlay("Isovist Points");
		ovl.clear();
		for (int i = 0; i < points.size(); i++) {
			ObservedLidarPointSlam point = points.get(i);
      ovl.fillCircle(point.x, point.y, 5, 0, 0, 0, 255);
		}
    ovl.paint();


		// Paint Isovist rays
		// ovl = Robot.debugPainter.getOverlay("Isovist Rays");
		// ovl.clear();
		// for (int i = 0; i < points.size(); i++) {
		// 	ObservedLidarPointSlam point = points.get(i);
		//  ovl.drawLine(lidarPackageSlam.observationPosX, lidarPackageSlam.observationPosY, point.x, point.y, 0, 255, 0, 100);
		// }
		// ovl.paint();


		ovl = Robot.debugPainter.getOverlay("Isovist Ploygon");
		ovl.clear();
		ovl.fillPoly(lidarPackageSlam.getPositionAsArray(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY), 0, 0, 255, 100);
		ovl.paint();

		////////////////////////////////////////////////////////////////////////////////

		int RAY_COUNT = 400;
		int MAX_DIST = 600;

		PointList2D<Point> gridPs = new ArrayPointList(360);
		
		ovl = Robot.debugPainter.getOverlay("Isovist Beam");
		ovl.clear();
		ovl.fillCircle(posX, posY, 10, 0, 0, 0, 255);
		for (int i = 0; i < RAY_COUNT; ++i) {
			double theta = ((float)i / RAY_COUNT) * 2 * Math.PI;
			double dx = Math.cos(theta) * MAX_DIST;
			double dy = Math.sin(theta) * MAX_DIST;

			ovl.drawLine(posX, posY, dx, dy, 0, 0, 0, 50);
			
			PointList2D<Point> ps = cloud.getInsideBeam(posX, posY, dx, dy, 10.0d);
			ps = ps.getKNearest(posX, posY, MAX_DIST, 1);

			// Point found!
			if (ps.size() > 0) {
				gridPs.add(ps.get(0));
			} else {
				// gridPs.add(new Point(dx, dy));
			}
			// for (Point p : ps)
			// 	ovl.fillCircle(p.getX(), p.getY(), 5, 0, 255, 0, 255);
		}

		for (Point p : gridPs) {
			ovl.fillCircle(p.getX(), p.getY(), 5, 0, 255, 0, 255);
		}
		ovl.paint();

		ovl = Robot.debugPainter.getOverlay("Grid Isovist Ploygon");
		ovl.clear();
		ovl.fillPoly(gridPs.getAll2D(), 0, 255, 0, 100);
		ovl.paint();
  }
}
