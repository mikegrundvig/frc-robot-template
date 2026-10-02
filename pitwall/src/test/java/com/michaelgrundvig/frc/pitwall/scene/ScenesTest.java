package com.michaelgrundvig.frc.pitwall.scene;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.push.Feed;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.Consumer;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * Scenes as the robot loop runs them: described once and checked whole, then only what moved by
 * what the page can show, at most 20 times a second, and a click answered from what was published.
 */
class ScenesTest {
  private static final JsonType<Object> JSON =
      Jsonb.builder().serializeNulls(true).build().type(Object.class);
  private static final long LOOP = 20_000_000L;

  private final AtomicLong clock = new AtomicLong(1_000_000_000L);
  private final List<String> said = new ArrayList<>();
  private final Scenes scenes = new Scenes(clock::get, said::add);

  /** Where the robot layer puts the robot, and how fast its wheel goes. */
  private Pose2d robot = new Pose2d(1, 2, Rotation2d.ZERO);

  private double speed;

  /** The rays layer's sight lines of the moment, and whether it starts them over this loop. */
  private List<String> seen = List.of();

  private boolean reseen;
  private @Nullable RuntimeException throwing;
  private @Nullable RuntimeException configurationThrows;
  private String camerasConfiguration = "mounted";

  private final Layer field =
      Layer.of(
          "field",
          "Field",
          scene -> {
            scene.frame("tag.7", "field", 4, 1, 0.5, 0, 0, Math.PI);
            scene
                .polyline("walls", "field")
                .point(0, 0, 0)
                .point(16, 0, 0)
                .point(16, 8, 0)
                .closed();
            scene.polygon("floor", "field").point(0, 0, 0).point(1, 0, 0).point(1, 1, 0).outline();
            scene
                .mesh("bump", "field")
                .vertex(0, 0, 0)
                .vertex(1, 0, 0)
                .vertex(0.5, 0, 0.2)
                .triangle(0, 1, 2);
            scene.label("tag.7.label", "tag.7").text("7").at(0, 0, 0.2);
          });

  private final Layer robotLayer =
      new Layer() {
        @Override
        public String id() {
          return "robot";
        }

        @Override
        public String title() {
          return "Robot";
        }

        @Override
        public void describe(SceneBuilder scene) {
          scene
              .frame("robot", "field")
              .frame(
                  "camera",
                  "robot",
                  new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-10), 0)))
              .value("speed", 0.05);
          scene.box("chassis", "robot").size(0.7, 0.6, 0.2).at(0, 0, 0.1).style(Style.ACCENT);
          scene
              .cylinder("wheel", "robot")
              .radius(0.05)
              .length(0.04)
              .at(0.3, 0.3, 0.05, Math.PI / 2, 0, 0);
          scene.arrow("drive", "robot").from(0.3, 0.3, 0.1).to(1, 0.3, 0.1).length("speed");
          scene
              .frustum("view", "camera")
              .fov(Math.toRadians(70), Math.toRadians(50))
              .range(0, 7)
              .inspect(() -> Inspection.of("camera").detail("Mount", camerasConfiguration));
        }

        @Override
        public void update(SceneState state) {
          RuntimeException problem = throwing;
          if (problem != null) {
            throw problem;
          }
          state.pose("robot", robot);
          state.value("speed", speed);
        }

        @Override
        public Object configuration() {
          RuntimeException problem = configurationThrows;
          if (problem != null) {
            throw problem;
          }
          return camerasConfiguration;
        }
      };

  private final Layer rays =
      new Layer() {
        @Override
        public String id() {
          return "rays";
        }

        @Override
        public String title() {
          return "Sight lines";
        }

        @Override
        public void describe(SceneBuilder scene) {}

        @Override
        public void update(SceneState state) {
          if (!reseen) {
            return;
          }
          reseen = false;
          Shapes shapes = state.shapes();
          for (String verdict : seen) {
            shapes
                .line("ray." + verdict, "field")
                .between("camera", "tag.7")
                .style(verdict.equals("ok") ? Style.OK : Style.BAD.dashed())
                .inspect(() -> Inspection.of("camera → tag 7").detail("Verdict", verdict));
          }
        }

        @Override
        public Optional<Inspection> inspect(String shapeId) {
          return shapeId.equals("walls")
              ? Optional.of(Inspection.of("The field's walls").action("Survey"))
              : Optional.empty();
        }
      };

  private Feed world() {
    scenes.add(
        Scene.named("world", "World")
            .layer(field)
            .layer(robotLayer)
            .layer(rays, Scene.Visibility.OFF)
            .focus("robot"));
    return scenes.feeds().getFirst();
  }

  /** One loop: the clock moves on, and the feed says what it sends. */
  private String loop(Feed feed) {
    clock.addAndGet(LOOP);
    return JSON.toJson(feed.patch());
  }

  private void see(String... verdicts) {
    seen = List.of(verdicts);
    reseen = true;
  }

  @Test
  void theDescriptionIsTheLayersFramesAndShapesAsThePageReadsThem() {
    world();
    String described = JSON.toJson(scenes.descriptions());

    assertThat(described)
        .startsWith(
            "{\"world\":{\"title\":\"World\",\"focus\":\"robot\","
                + "\"layers\":{\"field\":{\"title\":\"Field\",")
        .contains("\"rays\":{\"title\":\"Sight lines\",\"shown\":false")
        .contains("\"tag.7\":{\"parent\":\"field\",\"pose\":[4,1,0.5,0,0,180]}")
        .contains("\"camera\":{\"parent\":\"robot\",\"pose\":[0.3,0,0.5,0,-10,0]}")
        .contains(
            "{\"id\":\"walls\",\"kind\":\"polyline\",\"frame\":\"field\",\"style\":\"muted\","
                + "\"points\":[[0,0,0],[16,0,0],[16,8,0]],\"closed\":true}")
        .contains("\"kind\":\"polygon\"", "\"outline\":true")
        .contains("\"vertices\":[[0,0,0],[1,0,0],[0.5,0,0.2]],\"triangles\":[[0,1,2]]")
        .contains(
            "{\"id\":\"tag.7.label\",\"kind\":\"label\",\"frame\":\"tag.7\",\"style\":\"muted\","
                + "\"at\":[0,0,0.2,0,0,0],\"text\":\"7\"}")
        .contains(
            "{\"id\":\"chassis\",\"kind\":\"box\",\"frame\":\"robot\",\"style\":\"accent\","
                + "\"at\":[0,0,0.1,0,0,0],\"size\":[0.7,0.6,0.2]}")
        .contains("\"radius\":0.05,\"length\":0.04")
        .contains("\"from\":[0.3,0.3,0.1],\"to\":[1,0.3,0.1],\"length\":\"speed\"")
        .contains("\"inspect\":true,\"fov\":[70,50],\"range\":[0,7]");
    assertThat(scenes.descriptions())
        .as("the same, until a layer changes")
        .isSameAs(scenes.descriptions());
    assertThat(scenes.isEmpty()).isFalse();
    assertThat(scenes.has("world")).isTrue();
  }

  @Test
  void aStillRobotSendsNothingAndAMovingOneOnlyWhatMovedAStep() {
    Feed feed = world();
    assertThat(feed.at()).containsExactly("scenes", "world", "state");
    assertThat(loop(feed)).isEqualTo("{\"frames\":{\"robot\":[1,2,0]},\"values\":{\"speed\":0}}");
    for (int i = 0; i < 100; i++) {
      assertThat(loop(feed)).as("still").isEqualTo("{}");
    }

    robot = new Pose2d(1.0009, 2, Rotation2d.fromDegrees(0.09));
    speed = 0.04;
    for (int i = 0; i < 10; i++) {
      assertThat(loop(feed)).as("less than a step").isEqualTo("{}");
    }

    robot = new Pose2d(1.0012, 2, Rotation2d.ZERO);
    assertThat(loop(feed)).isEqualTo("{\"frames\":{\"robot\":[1.001,2,0]}}");
    robot = new Pose2d(1.0012, 2, Rotation2d.fromDegrees(-90.04));
    speed = 0.06;
    clock.addAndGet(SceneFeed.INTERVAL_NANOS);
    assertThat(loop(feed))
        .isEqualTo("{\"frames\":{\"robot\":[1.001,2,-90]},\"values\":{\"speed\":0.05}}");
  }

  @Test
  void patchesGoAtMostTwentyTimesASecondAndNothingWaitingIsLost() {
    Feed feed = world();
    List<String> sent = new ArrayList<>();
    for (int i = 0; i < 50; i++) { // one second of loops, the robot moving every loop
      robot = new Pose2d(1 + i * 0.01, 2, Rotation2d.ZERO);
      String patch = loop(feed);
      if (!patch.equals("{}")) {
        sent.add(patch);
      }
    }

    assertThat(sent).hasSizeLessThanOrEqualTo(20).hasSizeGreaterThanOrEqualTo(16);
    assertThat(sent.getLast()).contains("\"robot\":[1.").as("the latest pose, not a stale one");
    for (int i = 0; i < 3; i++) {
      loop(feed);
    }
    assertThat(JSON.toJson(feed.sent())).contains("\"robot\":[1.49,2,0]");
  }

  @Test
  void aLayersShapesOfTheMomentAreReplacedWholeAndKeptUntilStartedOver() {
    Feed feed = world();
    loop(feed);
    see("ok", "bad");
    clock.addAndGet(SceneFeed.INTERVAL_NANOS);
    assertThat(loop(feed))
        .isEqualTo(
            "{\"shapes\":{\"rays\":[{\"id\":\"ray.ok\",\"kind\":\"line\",\"frame\":\"field\","
                + "\"style\":\"ok\",\"inspect\":true,\"between\":[\"camera\",\"tag.7\"]},"
                + "{\"id\":\"ray.bad\",\"kind\":\"line\",\"frame\":\"field\",\"style\":\"bad dashed\","
                + "\"inspect\":true,\"between\":[\"camera\",\"tag.7\"]}]}}");
    clock.addAndGet(SceneFeed.INTERVAL_NANOS);
    assertThat(loop(feed)).as("kept: not started over").isEqualTo("{}");

    see("ok", "bad");
    clock.addAndGet(SceneFeed.INTERVAL_NANOS);
    assertThat(loop(feed)).as("started over, but the same").isEqualTo("{}");

    see();
    clock.addAndGet(SceneFeed.INTERVAL_NANOS);
    assertThat(loop(feed)).isEqualTo("{\"shapes\":{\"rays\":[]}}");
  }

  @Test
  void aShapeOfTheMomentCanBeInspectedForTwoSecondsAfterItsGone() {
    Feed feed = world();
    loop(feed);
    see("bad");
    loop(feed);
    assertThat(scenes.inspect("world", "ray.bad"))
        .hasValueSatisfying(
            inspection ->
                assertThat(inspection.details()).extracting("value").containsExactly("bad"));

    see();
    loop(feed);
    for (long t = 0; t < 1_900_000_000L; t += LOOP) {
      loop(feed);
    }
    assertThat(scenes.inspect("world", "ray.bad")).as("gone 1.9 s").isPresent();
    for (int i = 0; i < 10; i++) {
      loop(feed);
    }
    assertThat(scenes.inspect("world", "ray.bad")).as("gone over 2 s").isEmpty();
  }

  @Test
  void aShapeThatDoesntSayForItselfIsAskedOfItsLayers() {
    world();
    assertThat(scenes.inspect("world", "view"))
        .hasValueSatisfying(inspection -> assertThat(inspection.title()).isEqualTo("camera"));
    assertThat(scenes.inspect("world", "walls"))
        .hasValueSatisfying(
            inspection -> assertThat(inspection.actions()).containsExactly("Survey"));
    assertThat(scenes.inspect("world", "chassis")).isEmpty();
    assertThat(scenes.inspect("elsewhere", "walls")).isEmpty();
  }

  @Test
  void stylesChangeAndGoBackAsDescribed() {
    Layer styling =
        new Layer() {
          @Nullable Style style;

          @Override
          public String id() {
            return "styling";
          }

          @Override
          public String title() {
            return "Styling";
          }

          @Override
          public void describe(SceneBuilder scene) {}

          @Override
          public void update(SceneState state) {
            state.style("chassis", style);
            style = style == null ? Style.BAD.dashed() : null;
          }
        };
    scenes.add(Scene.named("world", "World").layer(field).layer(robotLayer).layer(styling));
    Feed feed = scenes.feeds().getFirst();

    assertThat(loop(feed)).doesNotContain("styles");
    clock.addAndGet(SceneFeed.INTERVAL_NANOS);
    assertThat(loop(feed)).contains("\"styles\":{\"chassis\":\"bad dashed\"}");
    clock.addAndGet(SceneFeed.INTERVAL_NANOS);
    assertThat(loop(feed)).contains("\"styles\":{\"chassis\":null}");
  }

  @Test
  void aPoseCanBeGivenAsWpilibHasIt() {
    Layer posing =
        new Layer() {
          @Override
          public String id() {
            return "posing";
          }

          @Override
          public String title() {
            return "Posing";
          }

          @Override
          public void describe(SceneBuilder scene) {
            scene
                .frame("level", "field")
                .frame("tilted", "field")
                .frame("tag", "field", new Pose3d(1, 2, 3, new Rotation3d(0, 0, Math.PI / 2)));
          }

          @Override
          public void update(SceneState state) {
            state.pose("level", 1, 2, Math.PI);
            state.pose("tilted", new Pose3d(1, 2, 0.1, new Rotation3d(Math.toRadians(5), 0, 0)));
            state.pose("level", Double.NaN, 0, 0);
          }
        };
    scenes.add(Scene.named("world", "World").layer(posing));

    assertThat(JSON.toJson(scenes.descriptions())).contains("\"pose\":[1,2,3,0,0,90]");
    assertThat(loop(scenes.feeds().getFirst()))
        .isEqualTo("{\"frames\":{\"level\":[1,2,180],\"tilted\":[1,2,0.1,5,0,0]}}");
  }

  @Test
  void aFrameWhosePoseWandersCanTakeAWiderDeadband() {
    Layer wandering =
        new Layer() {
          @Override
          public String id() {
            return "wandering";
          }

          @Override
          public String title() {
            return "Wandering";
          }

          @Override
          public void describe(SceneBuilder scene) {
            scene.frame("estimate", "field").deadband("estimate", 0.01, Math.toRadians(0.5));
          }

          @Override
          public void update(SceneState state) {
            state.pose("estimate", robot);
          }
        };
    scenes.add(Scene.named("world", "World").layer(wandering));
    Feed feed = scenes.feeds().getFirst();
    assertThat(loop(feed)).isEqualTo("{\"frames\":{\"estimate\":[1,2,0]}}");
    for (int i = 0; i < 20; i++) {
      robot = new Pose2d(1 + (i % 2 == 0 ? 0.004 : -0.004), 2, Rotation2d.fromDegrees(0.3));
      assertThat(loop(feed)).as("wandering within it").isEqualTo("{}");
    }

    robot = new Pose2d(1.012, 2, Rotation2d.ZERO);
    assertThat(loop(feed)).isEqualTo("{\"frames\":{\"estimate\":[1.012,2,0]}}");
    assertRefused(scene -> scene.deadband("elsewhere", 0.01, 0.01), "a frame this layer adds");
    assertRefused(
        scene -> scene.frame("f", "field").deadband("f", 0.0001, 0.01), "at least a millimeter");
  }

  @Test
  void aLayerDescribedAgainWhenItsConfigurationChanges() {
    Feed feed = world();
    Map<String, Object> before = scenes.descriptions();
    camerasConfiguration = "recalibrated";
    Map<String, Object> after = scenes.descriptions();

    assertThat(after).isNotSameAs(before);
    assertThat(scenes.inspect("world", "view"))
        .hasValueSatisfying(
            inspection ->
                assertThat(inspection.details())
                    .extracting("value")
                    .containsExactly("recalibrated"));
    assertThat(loop(feed)).contains("\"robot\":[1,2,0]");
  }

  @Test
  void aLayerThatThrowsCostsOnlyItselfAndIsReportedOnce() {
    Feed feed = world();
    throwing = new IllegalStateException("broken");
    see("ok");
    for (int i = 0; i < 5; i++) {
      loop(feed);
    }

    assertThat(said)
        .containsExactly(
            "Pitwall: scene world, layer robot couldn't update: java.lang.IllegalStateException:"
                + " broken");
    assertThat(JSON.toJson(feed.sent())).contains("ray.ok").doesNotContain("robot\":");
  }

  @Test
  void aLayerWhoseConfigurationThrowsCostsOnlyItselfAndIsReportedOnce() {
    Feed feed = world();
    Map<String, Object> before = scenes.descriptions();
    configurationThrows = new IllegalStateException("unreadable");
    for (int i = 0; i < 5; i++) {
      assertThat(scenes.descriptions()).as("the description before stands").isSameAs(before);
    }
    robot = new Pose2d(3, 2, Rotation2d.ZERO);

    assertThat(said)
        .containsExactly(
            "Pitwall: scene world, layer robot couldn't be described again:"
                + " java.lang.IllegalStateException: unreadable");
    assertThat(loop(feed)).as("its update still runs").contains("\"robot\":[3,2,0]");
    configurationThrows = null;
    camerasConfiguration = "recalibrated";
    assertThat(scenes.descriptions()).as("described again once it answers").isNotSameAs(before);
  }

  @Test
  void aShapeRevealsOthersWhileItsClickedAndTheyMustBeThere() {
    scenes.add(
        Scene.named("world", "World")
            .layer(
                Layer.of(
                    "robot",
                    "Robot",
                    scene -> {
                      scene.frame("camera", "field");
                      scene
                          .arrow("camera.axis", "camera")
                          .to(0.3, 0, 0)
                          .inspect(() -> Inspection.of("camera"))
                          .reveal("camera.view");
                    }))
            .layer(
                Layer.of(
                    "views",
                    "Camera views",
                    scene -> scene.frustum("camera.view", "camera").fov(1, 0.8).range(0, 7)),
                Scene.Visibility.OFF));

    assertThat(JSON.toJson(scenes.descriptions()))
        .contains("\"id\":\"camera.axis\"", "\"reveals\":[\"camera.view\"]");
    assertRefused(
        scene -> scene.box("b", "field").reveal("nothing"),
        "shape b reveals shape nothing, which the scene doesn't have");

    Layer moment =
        new Layer() {
          @Override
          public String id() {
            return "moment";
          }

          @Override
          public String title() {
            return "Moment";
          }

          @Override
          public void describe(SceneBuilder scene) {}

          @Override
          public void update(SceneState state) {
            state.shapes().line("ray", "field").to(1, 0, 0).reveal("ray", "gone");
          }
        };
    scenes.add(Scene.named("other", "Other").layer(moment));
    loop(scenes.feeds().getLast());
    assertThat(said).singleElement().asString().contains("reveals shape gone");
  }

  @Test
  void aNameTheSceneDoesntHaveInAnUpdateIsTheLayersMistake() {
    Layer typo =
        new Layer() {
          @Override
          public String id() {
            return "typo";
          }

          @Override
          public String title() {
            return "Typo";
          }

          @Override
          public void describe(SceneBuilder scene) {}

          @Override
          public void update(SceneState state) {
            state.pose("robto", 0, 0, 0);
          }
        };
    scenes.add(Scene.named("world", "World").layer(typo));
    loop(scenes.feeds().getFirst());

    assertThat(said).singleElement().asString().contains("has no frame named robto");
  }

  @Test
  void aSceneWhoseLayersDontFitTogetherIsRefusedWhenAdded() {
    assertRefused(scene -> scene.box("b", "nowhere"), "shape b is in frame nowhere");
    assertRefused(scene -> scene.frame("a", "b").frame("b", "a"), "is its own ancestor");
    assertRefused(scene -> scene.frame("a", "nothing"), "parent nothing isn't a frame");
    assertRefused(
        scene -> {
          scene.box("b", "field").size(1, 1, 1);
          scene.cylinder("b", "field");
        },
        "two shapes with the id b");
    assertRefused(
        scene -> scene.arrow("a", "field").to(1, 0, 0).length("speed"), "follows value speed");
    assertRefused(scene -> scene.frame("field", "field"), "root frame");
    assertRefused(scene -> scene.polyline("p", "field").point(0, 0, 0), "two points");
    assertThatThrownBy(
            () -> new Scenes(clock::get, said::add).add(Scene.named("s", "S").focus("robot")))
        .hasMessageContaining("follows frame robot, which it doesn't have");
    assertThatThrownBy(() -> Scene.named("world", "World").layer(field).layer(field))
        .hasMessageContaining("two layers named field");
    scenes.add(Scene.named("world", "World"));
    assertThatThrownBy(() -> scenes.add(Scene.named("world", "Again")))
        .hasMessageContaining("two scenes have the id world");
  }

  private void assertRefused(Consumer<SceneBuilder> describe, String why) {
    assertThatThrownBy(
            () ->
                new Scenes(clock::get, said::add)
                    .add(Scene.named("s", "S").layer(Layer.of("l", "L", describe))))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining(why);
  }
}
