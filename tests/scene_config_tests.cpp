#include <gtest/gtest.h>
#include <robot_simulation/scene_config.hpp>

namespace robot_simulation {
TEST(SceneConfigTests, ParsesCompleteDefaultConfiguration) {
    const json json_data = {
        {"environment", {{"width", 12.0}, {"length", 30.0}}},
        {"robot", {
            {"pose", {{"x", -8.0}, {"y", 0.0}, {"theta", 0.0}}},
            {"size", {{"width", 1.5}, {"length", 2.0}}},
            {"velocity", {{"linearVelocity", 2.0}, {"angularVelocity", 0.0}}}
        }},
        {"obstacles", {{{
            "pose", {{"x", 8.0}, {"y", 0.0}, {"theta", 0.0}}
        }, {
            "size", {{"width", 4.0}, {"length", 2.0}}
        }}}},
        {"simulation", {{"dt", 0.5}, {"ray_count", 3}}},
        {"controller", {
            {"safe_distance", 5.0},
            {"critical_distance", 2.0},
            {"velocity_gain_critical", 0.2}
        }}
    };

    const SceneConfig config = parseJsonToSimulationConfiguration(json_data);

    EXPECT_DOUBLE_EQ(config.environment_size.width, 12.0);
    EXPECT_DOUBLE_EQ(config.environment_size.length, 30.0);
    EXPECT_DOUBLE_EQ(config.robot.pose.x, -8.0);
    EXPECT_DOUBLE_EQ(config.robot.size.length, 2.0);
    EXPECT_DOUBLE_EQ(config.robot.velocity.linearVelocity, 2.0);
    ASSERT_EQ(config.obstacles.size(), 1U);
    EXPECT_DOUBLE_EQ(config.obstacles[0].pose.x, 8.0);
    EXPECT_DOUBLE_EQ(config.obstacles[0].size.width, 4.0);
    EXPECT_DOUBLE_EQ(config.dt, 0.5);
    EXPECT_EQ(config.ray_count, 3);
    EXPECT_DOUBLE_EQ(config.controller.safe_distance, 5.0);
    EXPECT_DOUBLE_EQ(config.controller.critical_distance, 2.0);
    EXPECT_DOUBLE_EQ(config.controller.velocity_gain_critical, 0.2);
}

TEST(SceneConfigTests, SerializesCompleteConfiguration) {
    const SceneConfig config {
        {12.0, 30.0},
        {{-8.0, 0.0, 0.0}, {1.5, 2.0}, {2.0, 0.0}},
        {{{8.0, 0.0, 0.0}, {4.0, 2.0}}, {{-2.0, 3.0, 1.0}, {1.0, 5.0}}},
        0.5,
        3,
        {5.0, 2.0, 0.2}
    };

    const json json_data = json::parse(ConfigurationToJson(config));

    EXPECT_EQ(json_data.at("environment").at("width"), 12.0);
    EXPECT_EQ(json_data.at("robot").at("velocity").at("linearVelocity"), 2.0);
    ASSERT_EQ(json_data.at("obstacles").size(), 2U);
    EXPECT_EQ(json_data.at("obstacles").at(1).at("pose").at("y"), 3.0);
    EXPECT_EQ(json_data.at("simulation").at("ray_count"), 3);
    EXPECT_EQ(json_data.at("controller").at("velocity_gain_critical"), 0.2);

    const SceneConfig parsed_config = parseJsonToSimulationConfiguration(json_data);
    EXPECT_DOUBLE_EQ(parsed_config.robot.size.width, config.robot.size.width);
    EXPECT_DOUBLE_EQ(parsed_config.obstacles[1].size.length, config.obstacles[1].size.length);
}
}  // namespace robot_simulation
