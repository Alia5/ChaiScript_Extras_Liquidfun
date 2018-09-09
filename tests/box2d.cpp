#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file

#include "catch.hpp"

#include <string>
#include <chaiscript/chaiscript.hpp>
#include <Box2D/Box2D.h>

/**
 * Load the Box2D library.
 */
#include "../include/chaiscript/extras/box2d.hpp"

TEST_CASE( "Box2D functions work", "[box2d]" ) {

  auto box2dlib = chaiscript::extras::box2d::bootstrap();

  // Create the ChaiScript environment.
  chaiscript::ChaiScript chai;

  // Add the library to the ChaiScript instance.
  chai.add(box2dlib);

  // b2Vec2
  chai.eval(R""(
    // Define the gravity vector.
    var gravity = b2Vec2(0.0f, -10.0f);
  )"");
  CHECK(chai.eval<float>("gravity.y") == -10.0f);

  // b2World
  chai.eval(R""(
    // Define the gravity vector.
    var world = b2World(gravity);
  )"");
  CHECK(chai.eval<bool>("world.GetSubStepping") == false);

  // b2BodyDef
  chai.eval(R""(
    // Define the ground body.
    var groundBodyDef = b2BodyDef()
    groundBodyDef.position.Set(0.0f, -10.0f)
  )"");
  CHECK(chai.eval<bool>("groundBodyDef.awake") == true);
}
