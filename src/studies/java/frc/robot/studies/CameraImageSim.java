package frc.robot.studies;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.TagDetections;
import frc.robot.subsystems.vision.VisionSim;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.opencv.core.Point;
import org.photonvision.estimation.CameraTargetRelation;
import org.photonvision.estimation.OpenCVHelp;
import org.photonvision.estimation.RotTrlTransform3d;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PnpResult;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.vision.camera.OpenCvLoader;

/**
 * A camera's images of the field's tags, taken as PhotonVision's simulation takes them ({@code
 * PhotonCameraSim.process}), with the camera model the simulator uses ({@link
 * VisionSim#properties}), step for step: which tags are in view, where their corners land in the
 * image, the same pixel noise from the same random draws, the same detection limits, and
 * PhotonVision's own solves. What it leaves out is the picture: PhotonVision draws every image into
 * native memory that only the garbage collector frees, which is harmless at a simulator's 30 images
 * a second, and exhausts the computer's memory at a study's hundreds of thousands. Its test checks
 * it against PhotonVision's own simulation over a sample of the field.
 *
 * <p>One per thread: its noise isn't thread-safe.
 */
final class CameraImageSim {
  /** The smallest tag PhotonVision's simulation detects, in square pixels: its default. */
  static final double MIN_AREA_PIXELS = 100;

  private final SimCameraProperties camera;
  private final double fx;
  private final double fy;
  private final double cx;
  private final double cy;
  private final double halfWidthDegrees;
  private final double halfHeightDegrees;
  private final List<Tag> tags = new ArrayList<>();

  /** A tag on the field, and where its corners are, in PhotonVision's order. */
  private record Tag(int id, Pose3d pose, List<Translation3d> corners) {}

  /** A tag in an image: where its corners landed, noise and all, and its pose solved from them. */
  private record Seen(Tag tag, Point[] corners, PnpResult solved) {}

  CameraImageSim(FieldModel field, CameraConfig.Sensor sensor) {
    OpenCvLoader.forceStaticLoad(); // the solves are OpenCV's
    camera = VisionSim.properties(sensor, 0);
    var intrinsics = camera.getIntrinsics();
    fx = intrinsics.get(0, 0);
    fy = intrinsics.get(1, 1);
    cx = intrinsics.get(0, 2);
    cy = intrinsics.get(1, 2);
    halfWidthDegrees = camera.getHorizFOV().getDegrees() / 2;
    halfHeightDegrees = camera.getVertFOV().getDegrees() / 2;
    for (Map.Entry<Integer, Pose3d> tag : field.tags().entrySet()) {
      tags.add(
          new Tag(
              tag.getKey(),
              tag.getValue(),
              TargetModel.kAprilTag36h11.getFieldVertices(tag.getValue())));
    }
  }

  /**
   * One image from a camera at a place on the field, with its own pixel noise, and the tags in it
   * as the robot reads them ({@code CameraIOPhotonVision}, then {@code Vision}).
   *
   * @param cameraPose where the camera is on the field, and which way it looks
   * @param cameraName which camera took it, as the vision pipeline knows it
   * @param seed where this image's noise starts
   */
  TagDetections image(Pose3d cameraPose, String cameraName, long seed) {
    camera.setRandomSeed(seed);
    RotTrlTransform3d toCamera = RotTrlTransform3d.makeRelativeTo(cameraPose);
    // Farthest first, as PhotonVision takes them, so each tag's noise comes from the same draws.
    List<Tag> farthestFirst = new ArrayList<>(tags);
    farthestFirst.sort(
        Comparator.comparingDouble(
                (Tag tag) -> tag.pose().getTranslation().getDistance(cameraPose.getTranslation()))
            .reversed());
    List<Seen> seen = new ArrayList<>();
    for (Tag tag : farthestFirst) {
      if (!inView(cameraPose, tag)) {
        continue;
      }
      Point[] corners = camera.estPixelNoise(project(toCamera, tag.corners()));
      if (!inImage(corners) || area(corners) < MIN_AREA_PIXELS) {
        continue;
      }
      PnpResult solved =
          OpenCVHelp.solvePNP_SQUARE(
                  camera.getIntrinsics(),
                  camera.getDistCoeffs(),
                  TargetModel.kAprilTag36h11.vertices,
                  corners)
              .orElseGet(PnpResult::new);
      seen.add(new Seen(tag, corners, solved));
    }
    List<TagDetections.Tag> detected = detected(seen);
    return new TagDetections(0, cameraName, detected, multiTag(seen, detected));
  }

  /**
   * Whether a tag is in view, by PhotonVision's test: its center within the field of view, its face
   * toward the camera, and near enough to detect.
   */
  private boolean inView(Pose3d cameraPose, Tag tag) {
    CameraTargetRelation relation = new CameraTargetRelation(cameraPose, tag.pose());
    return Math.abs(relation.camToTargYaw.getDegrees()) < halfWidthDegrees
        && Math.abs(relation.camToTargPitch.getDegrees()) < halfHeightDegrees
        && Math.abs(relation.targToCamAngle.getDegrees()) < 90
        && relation.camToTarg.getTranslation().getNorm() <= VisionSim.MAX_SIGHT_METERS;
  }

  /**
   * Where points on the field land in the image: a pinhole camera, as OpenCV projects them for
   * PhotonVision, with a perfect lens.
   */
  private Point[] project(RotTrlTransform3d toCamera, List<Translation3d> points) {
    Point[] image = new Point[points.size()];
    for (int i = 0; i < image.length; i++) {
      // The camera's frame is x out of the lens, y left, z up; the image's, right and down.
      Translation3d p = toCamera.apply(points.get(i));
      image[i] = new Point(cx - fx * p.getY() / p.getX(), cy - fy * p.getZ() / p.getX());
    }
    return image;
  }

  /** Whether every corner is within the image, edges included. */
  private boolean inImage(Point[] corners) {
    for (Point corner : corners) {
      if (!(corner.x >= 0
          && corner.x <= camera.getResWidth()
          && corner.y >= 0
          && corner.y <= camera.getResHeight())) {
        return false;
      }
    }
    return true;
  }

  /** The area of the corners' convex hull, in square pixels, as PhotonVision measures a tag. */
  static double area(Point[] corners) {
    List<Point> hull = hull(corners);
    double twice = 0;
    for (int i = 0; i < hull.size(); i++) {
      Point a = hull.get(i);
      Point b = hull.get((i + 1) % hull.size());
      twice += a.x * b.y - b.x * a.y;
    }
    return Math.abs(twice) / 2;
  }

  /** The convex hull, by Andrew's monotone chain: its lower half, then its upper. */
  private static List<Point> hull(Point[] points) {
    List<Point> sorted = new ArrayList<>(List.of(points));
    sorted.sort(Comparator.comparingDouble((Point p) -> p.x).thenComparingDouble(p -> p.y));
    List<Point> hull = new ArrayList<>();
    for (int half = 0; half < 2; half++) {
      int start = hull.size();
      for (Point p : sorted) {
        while (hull.size() >= start + 2
            && cross(hull.get(hull.size() - 2), hull.get(hull.size() - 1), p) <= 0) {
          hull.remove(hull.size() - 1);
        }
        hull.add(p);
      }
      hull.remove(hull.size() - 1); // the other half starts with it
      sorted = sorted.reversed();
    }
    return hull;
  }

  private static double cross(Point o, Point a, Point b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  }

  /** Each tag seen, as the robot reads it; one whose pose isn't a number is left out. */
  private static List<TagDetections.Tag> detected(List<Seen> seen) {
    List<TagDetections.Tag> detected = new ArrayList<>();
    for (Seen tag : seen) {
      if (!isFinite(tag.solved().best) || !isFinite(tag.solved().alt)) {
        continue;
      }
      List<Translation2d> corners = new ArrayList<>();
      for (Point corner : tag.corners()) {
        corners.add(new Translation2d(corner.x, corner.y));
      }
      double ambiguity = tag.solved().ambiguity;
      detected.add(
          new TagDetections.Tag(
              tag.tag().id(),
              corners,
              tag.solved().best,
              tag.solved().alt,
              // PhotonVision's -1 means it has none to give: a coin toss, as Vision reads it.
              !(ambiguity >= 0) ? 1 : Math.min(ambiguity, 1)));
    }
    return detected;
  }

  /**
   * PhotonVision's solve from every tag in the image at once, when there are two or more: the
   * camera's pose on the field, from all their corners. It names only the tags the robot reads, as
   * {@code Vision} does.
   */
  private TagDetections.@Nullable MultiTag multiTag(
      List<Seen> seen, List<TagDetections.Tag> detected) {
    if (seen.size() < 2 || detected.isEmpty()) {
      return null;
    }
    List<Translation3d> field = new ArrayList<>();
    List<Point> image = new ArrayList<>();
    for (Seen tag : seen) {
      field.addAll(tag.tag().corners());
      image.addAll(List.of(tag.corners()));
    }
    PnpResult solved =
        OpenCVHelp.solvePNP_SQPNP(
                camera.getIntrinsics(), camera.getDistCoeffs(), field, image.toArray(Point[]::new))
            .orElse(null);
    if (solved == null || !isFinite(solved.best) || !Double.isFinite(solved.bestReprojErr)) {
      return null;
    }
    return new TagDetections.MultiTag(
        solved.best.inverse(),
        Math.max(0, solved.bestReprojErr),
        detected.stream().map(TagDetections.Tag::id).toList());
  }

  private static boolean isFinite(Transform3d transform) {
    Quaternion rotation = transform.getRotation().getQuaternion();
    return Double.isFinite(transform.getX())
        && Double.isFinite(transform.getY())
        && Double.isFinite(transform.getZ())
        && Double.isFinite(rotation.getW())
        && Double.isFinite(rotation.getX())
        && Double.isFinite(rotation.getY())
        && Double.isFinite(rotation.getZ());
  }
}
