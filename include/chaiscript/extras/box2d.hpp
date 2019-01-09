#include <vector>
#include <string>

#include <chaiscript/chaiscript.hpp>

#include <Box2D/Box2D.h>

namespace chaiscript {
  namespace extras {
    namespace box2d {

      /**
       * Register a ChaiScript module with b2BlockAllocator.
       */
      ModulePtr addb2BlockAllocator(ModulePtr m = std::make_shared<Module>()) {
        // b2BlockAllocator
        m->add(user_type<b2BlockAllocator>(), "b2BlockAllocator");
        m->add(constructor<b2BlockAllocator()>(), "b2BlockAllocator");
        m->add(fun(&b2BlockAllocator::Clear), "Clear");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Draw.
       */
      ModulePtr addb2Draw(ModulePtr m = std::make_shared<Module>()) {
        // b2Color
        m->add(user_type<b2Color>(), "b2Color");
        m->add(constructor<b2Color()>(), "b2Color");
        m->add(constructor<b2Color(float32, float32, float32)>(), "b2Color");
        m->add(fun(&b2Color::Set), "Set");

        // b2Draw
        m->add(user_type<b2Draw>(), "b2Draw");
        // m->add(constructor<b2Draw()>(), "b2Draw");
        m->add(fun(&b2Draw::SetFlags), "Set");
        m->add(fun(&b2Draw::GetFlags), "GetFlags");
        m->add(fun(&b2Draw::AppendFlags), "AppendFlags");
        m->add(fun(&b2Draw::ClearFlags), "ClearFlags");
        m->add(fun(&b2Draw::DrawPolygon), "DrawPolygon");
        m->add(fun(&b2Draw::DrawSolidPolygon), "DrawSolidPolygon");
        m->add(fun(&b2Draw::DrawCircle), "DrawCircle");
        m->add(fun(&b2Draw::DrawSolidCircle), "DrawSolidCircle");
        m->add(fun(&b2Draw::DrawSegment), "DrawSegment");
        m->add(fun(&b2Draw::DrawTransform), "DrawTransform");

        return m;
      }

      /**
       * Register a ChaiScript module with b2StackAllocator.
       */
      ModulePtr addb2StackAllocator(ModulePtr m = std::make_shared<Module>()) {
        // b2StackEntry
        m->add(user_type<b2StackEntry>(), "b2StackEntry");
        m->add(fun(&b2StackEntry::data), "data");
        m->add(fun(&b2StackEntry::size), "size");
        m->add(fun(&b2StackEntry::usedMalloc), "usedMalloc");

        // b2StackAllocator
        m->add(user_type<b2StackAllocator>(), "b2StackAllocator");
        m->add(constructor<b2StackAllocator()>(), "b2StackAllocator");
        m->add(fun(&b2StackAllocator::GetMaxAllocation), "GetMaxAllocation");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Timer.
       *
       * @see b2Timer.h
       */
      ModulePtr addb2Timer(ModulePtr m = std::make_shared<Module>()) {
        // b2Timer
        m->add(user_type<b2Timer>(), "b2Timer");
        m->add(constructor<b2Timer()>(), "b2Timer");
        m->add(fun(&b2Timer::Reset), "Reset");
        m->add(fun(&b2Timer::GetMilliseconds), "GetMilliseconds");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Body.
       */
      ModulePtr addb2Body(ModulePtr m = std::make_shared<Module>()) {

        // b2BodyType
        m->add(chaiscript::type_conversion<b2BodyType, int>());
        m->add_global_const(const_var(b2_staticBody), "b2_staticBody");
        m->add_global_const(const_var(b2_kinematicBody), "b2_kinematicBody");
        m->add_global_const(const_var(b2_dynamicBody), "b2_dynamicBody");
        m->add(chaiscript::type_conversion<const b2BodyType, b2BodyType>());
        m->add(fun([](b2BodyType& left, const b2BodyType& right) -> b2BodyType& {
          return (left = right);
        }), "=");

        // b2BodyDef
        m->add(user_type<b2BodyDef>(), "b2BodyDef");
        m->add(constructor<b2BodyDef()>(), "b2BodyDef");
        m->add(fun(&b2BodyDef::type), "type");
        m->add(fun(&b2BodyDef::position), "position");
        m->add(fun(&b2BodyDef::angle), "angle");
        m->add(fun(&b2BodyDef::linearVelocity), "linearVelocity");
        m->add(fun(&b2BodyDef::angularVelocity), "angularVelocity");
        m->add(fun(&b2BodyDef::linearDamping), "linearDamping");
        m->add(fun(&b2BodyDef::angularDamping), "angularDamping");
        m->add(fun(&b2BodyDef::allowSleep), "allowSleep");
        m->add(fun(&b2BodyDef::awake), "awake");
        m->add(fun(&b2BodyDef::fixedRotation), "fixedRotation");
        m->add(fun(&b2BodyDef::bullet), "bullet");
        m->add(fun(&b2BodyDef::active), "active");
        m->add(fun(&b2BodyDef::gravityScale), "gravityScale");

        // b2Body
        m->add(user_type<b2Body>(), "b2Body");
        m->add(fun(static_cast<b2Fixture*( b2Body::*)(const b2FixtureDef*)>(&b2Body::CreateFixture)), "CreateFixture");
        m->add(fun(static_cast<b2Fixture*( b2Body::*)(const b2Shape*, float32)>(&b2Body::CreateFixture)), "CreateFixture");
        m->add(fun(&b2Body::DestroyFixture), "DestroyFixture");
        m->add(fun(&b2Body::SetTransform), "SetTransform");
        m->add(fun(&b2Body::GetTransform), "GetTransform");
        m->add(fun(&b2Body::GetPosition), "GetPosition");
        m->add(fun(&b2Body::GetAngle), "GetAngle");
        m->add(fun(&b2Body::GetWorldCenter), "GetWorldCenter");
        m->add(fun(&b2Body::GetLocalCenter), "GetLocalCenter");
        m->add(fun(&b2Body::SetLinearVelocity), "SetLinearVelocity");
        m->add(fun(&b2Body::GetLinearVelocity), "GetLinearVelocity");
        m->add(fun(&b2Body::SetAngularVelocity), "SetAngularVelocity");
        m->add(fun(&b2Body::GetAngularVelocity), "GetAngularVelocity");
        m->add(fun(&b2Body::ApplyForce), "ApplyForce");
        m->add(fun(&b2Body::ApplyForceToCenter), "ApplyForceToCenter");
        m->add(fun(&b2Body::ApplyTorque), "ApplyTorque");
        m->add(fun(&b2Body::ApplyLinearImpulse), "ApplyLinearImpulse");
        //m->add(fun(&b2Body::ApplyLinearImpulseToCenter), "ApplyLinearImpulseToCenter");
        m->add(fun(&b2Body::ApplyAngularImpulse), "ApplyAngularImpulse");
        m->add(fun(&b2Body::GetMass), "GetMass");
        m->add(fun(&b2Body::GetInertia), "GetInertia");
        m->add(fun(&b2Body::GetMassData), "GetMassData");
        m->add(fun(&b2Body::SetMassData), "SetMassData");
        m->add(fun(&b2Body::ResetMassData), "ResetMassData");
        m->add(fun(&b2Body::GetWorldPoint), "GetWorldPoint");
        m->add(fun(&b2Body::GetWorldVector), "GetWorldVector");
        m->add(fun(&b2Body::GetLocalPoint), "GetLocalPoint");
        m->add(fun(&b2Body::GetLocalVector), "GetLocalVector");
        m->add(fun(&b2Body::GetLinearVelocityFromWorldPoint), "GetLinearVelocityFromWorldPoint");
        m->add(fun(&b2Body::GetLinearVelocityFromLocalPoint), "GetLinearVelocityFromLocalPoint");
        m->add(fun(&b2Body::GetLinearDamping), "GetLinearDamping");
        m->add(fun(&b2Body::SetLinearDamping), "SetLinearDamping");
        m->add(fun(&b2Body::GetAngularDamping), "GetAngularDamping");
        m->add(fun(&b2Body::SetAngularDamping), "SetAngularDamping");
        m->add(fun(&b2Body::GetGravityScale), "GetGravityScale");
        m->add(fun(&b2Body::SetGravityScale), "SetGravityScale");
        m->add(fun(&b2Body::SetType), "SetType");
        m->add(fun(&b2Body::GetType), "GetType");
        m->add(fun(&b2Body::SetBullet), "SetBullet");
        m->add(fun(&b2Body::IsBullet), "IsBullet");
        m->add(fun(&b2Body::SetSleepingAllowed), "SetSleepingAllowed");
        m->add(fun(&b2Body::IsSleepingAllowed), "IsSleepingAllowed");
        m->add(fun(&b2Body::SetAwake), "SetAwake");
        m->add(fun(&b2Body::IsAwake), "IsAwake");
        m->add(fun(&b2Body::SetActive), "SetActive");
        m->add(fun(&b2Body::IsActive), "IsActive");
        m->add(fun(&b2Body::SetFixedRotation), "SetFixedRotation");
        m->add(fun(&b2Body::IsFixedRotation), "IsFixedRotation");
        m->add(fun<b2Fixture*, b2Body>(&b2Body::GetFixtureList), "GetFixtureList");
        m->add(fun<b2JointEdge*, b2Body>(&b2Body::GetJointList), "GetJointList");
        m->add(fun<b2ContactEdge*, b2Body>(&b2Body::GetContactList), "GetContactList");
        m->add(fun<b2Body*, b2Body>(&b2Body::GetNext), "GetNext");
        // TODO: Figure out a way to pass void* as an argument through ChaiScript.
        // m->add(fun(&b2Body::GetUserData), "GetUserData");
        // m->add(fun(&b2Body::SetUserData), "SetUserData");
        m->add(fun<b2World*, b2Body>(&b2Body::GetWorld), "GetWorld");
        m->add(fun(&b2Body::Dump), "Dump");

        return m;
      }

      /**
       * Register a ChaiScript module with b2ContactManager.
       *
       * @see b2ContactManager.h
       */
      ModulePtr addb2ContactManager(ModulePtr m = std::make_shared<Module>()) {
        m->add(user_type<b2ContactManager>(), "b2ContactManager");
        m->add(constructor<b2ContactManager()>(), "b2ContactManager");
        m->add(fun(&b2ContactManager::FindNewContacts), "FindNewContacts");
        m->add(fun(&b2ContactManager::Collide), "Collide");

        return m;
      }

      /**
       * Register a ChaiScript module with b2PolygonShape.
       *
       * @see b2PolygonShape.h
       */
      ModulePtr addb2PolygonShape(ModulePtr m = std::make_shared<Module>()) {
        // b2PolygonShape
        m->add(user_type<b2PolygonShape>(), "b2PolygonShape");
        m->add(chaiscript::base_class<b2Shape, b2PolygonShape>());
        m->add(constructor<b2PolygonShape()>(), "b2PolygonShape");
        m->add(fun(&b2PolygonShape::GetChildCount), "GetChildCount");
        m->add(fun(&b2PolygonShape::Set), "Set");
        // TODO: Add b2BlockAllocator and b2PolygonShape::Clone
        //b2Shape* Clone(b2BlockAllocator* allocator) const override;
        m->add(fun(static_cast<void( b2PolygonShape::*)(float32, float32)>(&b2PolygonShape::SetAsBox)), "SetAsBox");
        m->add(fun(static_cast<void( b2PolygonShape::*)(float32, float32, const b2Vec2&, float32)>(&b2PolygonShape::SetAsBox)), "SetAsBox");
        m->add(fun(&b2PolygonShape::TestPoint), "TestPoint");
        m->add(fun(&b2PolygonShape::RayCast), "RayCast");
        m->add(fun(&b2PolygonShape::ComputeAABB), "ComputeAABB");
        m->add(fun(&b2PolygonShape::ComputeMass), "ComputeMass");
        m->add(fun(&b2PolygonShape::Validate), "Validate");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Fixture.
       *
       * @see b2Fixture.h
       */
      ModulePtr addb2Fixture(ModulePtr m = std::make_shared<Module>()) {
        // b2Filter
        m->add(user_type<b2Filter>(), "b2Filter");
        m->add(constructor<b2Filter()>(), "b2Filter");
        m->add(fun(&b2Filter::categoryBits), "categoryBits");
        m->add(fun(&b2Filter::maskBits), "maskBits");
        m->add(fun(&b2Filter::groupIndex), "groupIndex");

        // b2FixtureDef
        m->add(user_type<b2FixtureDef>(), "b2FixtureDef");
        m->add(constructor<b2FixtureDef()>(), "b2FixtureDef");
        m->add(fun(&b2FixtureDef::shape), "shape");
        m->add(fun(&b2FixtureDef::friction), "friction");
        m->add(fun(&b2FixtureDef::restitution), "restitution");
        m->add(fun(&b2FixtureDef::density), "density");
        m->add(fun(&b2FixtureDef::isSensor), "isSensor");
        m->add(fun(&b2FixtureDef::filter), "filter");
        // TODO: Figure out assignment of b2FixtureDef::shape.
        m->add(fun([](b2FixtureDef* f, b2Shape* s) {
          f->shape = s;
        }), "SetShape");

        // b2FixtureProxy
        m->add(user_type<b2FixtureProxy>(), "b2FixtureProxy");
        m->add(fun(&b2FixtureProxy::aabb), "aabb");
        m->add(fun(&b2FixtureProxy::fixture), "fixture");
        m->add(fun(&b2FixtureProxy::childIndex), "childIndex");
        m->add(fun(&b2FixtureProxy::proxyId), "proxyId");

        // b2Fixture
        m->add(user_type<b2Fixture>(), "b2Fixture");
        m->add(fun(&b2Fixture::GetType), "GetType");
        m->add(fun<b2Shape*, b2Fixture>(&b2Fixture::GetShape), "GetShape");
        m->add(fun(&b2Fixture::SetSensor), "SetSensor");
        m->add(fun(&b2Fixture::IsSensor), "IsSensor");
        m->add(fun(&b2Fixture::SetFilterData), "SetFilterData");
        m->add(fun(&b2Fixture::GetFilterData), "GetFilterData");
        m->add(fun(&b2Fixture::Refilter), "Refilter");
        m->add(fun<b2Body*, b2Fixture>(&b2Fixture::GetBody), "GetBody");
        m->add(fun<b2Fixture*, b2Fixture>(&b2Fixture::GetNext), "GetNext");
        //m->add(fun(&b2Fixture::GetUserData), "GetUserData");
        //m->add(fun(&b2Fixture::SetUserData), "SetUserData");
        m->add(fun(&b2Fixture::TestPoint), "TestPoint");
        m->add(fun(&b2Fixture::RayCast), "RayCast");
        m->add(fun(&b2Fixture::GetMassData), "GetMassData");
        m->add(fun(&b2Fixture::SetDensity), "SetDensity");
        m->add(fun(&b2Fixture::GetDensity), "GetDensity");
        m->add(fun(&b2Fixture::GetFriction), "GetFriction");
        m->add(fun(&b2Fixture::SetFriction), "SetFriction");
        m->add(fun(&b2Fixture::GetRestitution), "GetRestitution");
        m->add(fun(&b2Fixture::SetRestitution), "SetRestitution");
        m->add(fun(&b2Fixture::GetAABB), "GetAABB");
        m->add(fun(&b2Fixture::Dump), "Dump");

        return m;
      }

      /**
       * Register a ChaiScript module with b2World.
       *
       * @see b2World.h
       */
      ModulePtr addb2World(ModulePtr m = std::make_shared<Module>()) {
        // b2World
        m->add(user_type<b2World>(), "b2World");
        m->add(constructor<b2World(const b2Vec2&)>(), "b2World");
        m->add(fun(&b2World::SetDestructionListener), "SetDestructionListener");
        m->add(fun(&b2World::SetContactFilter), "SetContactFilter");
        m->add(fun(&b2World::SetContactListener), "SetContactListener");
        m->add(fun(&b2World::SetDebugDraw), "SetDebugDraw");
        m->add(fun(&b2World::CreateBody), "CreateBody");
        m->add(fun(&b2World::DestroyBody), "DestroyBody");
        m->add(fun(&b2World::CreateJoint), "CreateJoint");
        m->add(fun(&b2World::DestroyJoint), "DestroyJoint");
        m->add(fun(&b2World::CreateParticleSystem), "CreateParticleSystem");
        m->add(fun(&b2World::DestroyParticleSystem), "DestroyParticleSystem");
        m->add(fun(static_cast<void(b2World::*)(float32, int32, int32, int32)>(&b2World::Step)), "Step");
        m->add(fun(static_cast<void(b2World::*)(float32, int32, int32)>(&b2World::Step)), "Step");
        m->add(fun(&b2World::CalculateReasonableParticleIterations), "CalculateReasonableParticleIterations");
        m->add(fun(&b2World::ClearForces), "ClearForces");
        m->add(fun(&b2World::DrawDebugData), "DrawDebugData");
        m->add(fun(&b2World::QueryAABB), "QueryAABB");
        m->add(fun(&b2World::QueryShapeAABB), "QueryShapeAABB");
        m->add(fun(&b2World::RayCast), "RayCast");
        m->add(fun(static_cast<b2Body*( b2World::*)()>(&b2World::GetBodyList)), "GetBodyList");
        m->add(fun(static_cast<const b2Body*( b2World::*)() const>(&b2World::GetBodyList)), "GetBodyList");
        m->add(fun(static_cast<b2Joint*(b2World::*)()>(&b2World::GetJointList)), "GetJointList");
        m->add(fun(static_cast<const b2Joint*(b2World::*)() const>(&b2World::GetJointList)), "GetJointList");
        m->add(fun(static_cast<b2ParticleSystem*(b2World::*)()>(&b2World::GetParticleSystemList)), "GetParticleSystemList");
        m->add(fun(static_cast<const b2ParticleSystem*(b2World::*)() const>(&b2World::GetParticleSystemList)), "GetParticleSystemList");
        m->add(fun(static_cast<b2Contact*(b2World::*)()>(&b2World::GetContactList)), "GetContactList");
        m->add(fun(static_cast<const b2Contact*(b2World::*)() const>(&b2World::GetContactList)), "GetContactList");
        m->add(fun(&b2World::SetAllowSleeping), "SetAllowSleeping");
        m->add(fun(&b2World::GetAllowSleeping), "GetAllowSleeping");
        m->add(fun(&b2World::SetWarmStarting), "SetWarmStarting");
        m->add(fun(&b2World::GetWarmStarting), "GetWarmStarting");
        m->add(fun(&b2World::SetContinuousPhysics), "SetContinuousPhysics");
        m->add(fun(&b2World::GetContinuousPhysics), "GetContinuousPhysics");
        m->add(fun(&b2World::SetSubStepping), "SetSubStepping");
        m->add(fun(&b2World::GetSubStepping), "GetSubStepping");
        m->add(fun(&b2World::GetProxyCount), "GetProxyCount");
        m->add(fun(&b2World::GetBodyCount), "GetBodyCount");
        m->add(fun(&b2World::GetJointCount), "GetJointCount");
        m->add(fun(&b2World::GetContactCount), "GetContactCount");
        m->add(fun(&b2World::GetTreeHeight), "GetTreeHeight");
        m->add(fun(&b2World::GetTreeBalance), "GetTreeBalance");
        m->add(fun(&b2World::GetTreeQuality), "GetTreeQuality");
        m->add(fun(&b2World::SetGravity), "SetGravity");
        m->add(fun(&b2World::GetGravity), "GetGravity");
        m->add(fun(&b2World::IsLocked), "IsLocked");
        m->add(fun(&b2World::SetAutoClearForces), "SetAutoClearForces");
        m->add(fun(&b2World::GetAutoClearForces), "GetAutoClearForces");
        m->add(fun(&b2World::ShiftOrigin), "ShiftOrigin");
        m->add(fun(&b2World::GetContactManager), "GetContactManager");
        m->add(fun(&b2World::GetProfile), "GetProfile");
        m->add(fun(&b2World::Dump), "Dump");
        m->add(fun(&b2World::GetVersion), "GetVersion");
        m->add(fun(&b2World::GetVersionString), "GetVersionString");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Particle
       */
      ModulePtr addb2Particle(ModulePtr m = std::make_shared<Module>())
      {
        // b2ParticleFlag
        m->add(chaiscript::type_conversion<b2ParticleFlag, int>());
        m->add_global_const(const_var(b2_waterParticle), "b2_waterParticle");
        m->add_global_const(const_var(b2_zombieParticle), "b2_zombieParticle");
        m->add_global_const(const_var(b2_wallParticle), "b2_wallParticle");
        m->add_global_const(const_var(b2_springParticle), "b2_springParticle");
        m->add_global_const(const_var(b2_elasticParticle), "b2_elasticParticle");
        m->add_global_const(const_var(b2_viscousParticle), "b2_viscousParticle");
        m->add_global_const(const_var(b2_powderParticle), "b2_powderParticle");
        m->add_global_const(const_var(b2_tensileParticle), "b2_tensileParticle");
        m->add_global_const(const_var(b2_colorMixingParticle), "b2_colorMixingParticle");
        m->add_global_const(const_var(b2_destructionListenerParticle), "b2_destructionListenerParticle");
        m->add_global_const(const_var(b2_barrierParticle), "b2_barrierParticle");
        m->add_global_const(const_var(b2_staticPressureParticle), "b2_staticPressureParticle");
        m->add_global_const(const_var(b2_reactiveParticle), "b2_reactiveParticle");
        m->add_global_const(const_var(b2_repulsiveParticle), "b2_repulsiveParticle");
        m->add_global_const(const_var(b2_fixtureContactListenerParticle), "b2_fixtureContactListenerParticle");
        m->add_global_const(const_var(b2_particleContactListenerParticle), "b2_particleContactListenerParticle");
        m->add_global_const(const_var(b2_fixtureContactFilterParticle), "b2_fixtureContactFilterParticle");
        m->add_global_const(const_var(b2_particleContactFilterParticle), "b2_particleContactFilterParticle");
        m->add(chaiscript::type_conversion<const b2ParticleFlag, b2ParticleFlag>());
        m->add(fun([](b2ParticleFlag& left, const b2ParticleFlag& right) -> b2ParticleFlag& {
          return (left = right);
        }), "=");

        // b2ParticleColor
        m->add(user_type<b2ParticleColor>(), "b2ParticleColor");
        m->add(constructor<b2ParticleColor()>(), "b2ParticleColor");
        m->add(constructor<b2ParticleColor(uint8, uint8, uint8, uint8)>(), "b2ParticleColor");
        m->add(constructor<b2ParticleColor(const b2Color&)>(), "b2ParticleColor");
        m->add(fun(&b2ParticleColor::IsZero), "IsZero");
        m->add(fun(&b2ParticleColor::GetColor), "GetColor");
        m->add(fun(static_cast<void(b2ParticleColor::*)(uint8, uint8, uint8, uint8)>(&b2ParticleColor::Set)), "Set");
        m->add(fun(static_cast<void(b2ParticleColor::*)(const b2Color&)>(&b2ParticleColor::Set)), "Set");
        m->add(fun(static_cast<b2ParticleColor&(b2ParticleColor::*)(const b2ParticleColor&)>(&b2ParticleColor::operator=)), "=");
        m->add(fun([](const b2ParticleColor& color, float32 s)
        {
          return color * s;
        }), "*");
        m->add(fun([](const b2ParticleColor& color, uint8 s)
        {
          return color * s;
        }), "*");
        m->add(fun([](const b2ParticleColor& lhs, const b2ParticleColor& rhs)
        {
          return lhs + rhs;
        }), "+");
        m->add(fun([](const b2ParticleColor& lhs, const b2ParticleColor& rhs)
        {
          return lhs - rhs;
        }), "-");
        m->add(fun([](const b2ParticleColor& lhs, const b2ParticleColor& rhs)
        {
          return lhs == rhs;
        }), "==");
        m->add(fun(static_cast<void(b2ParticleColor::*)(b2ParticleColor*const, const int32)>(&b2ParticleColor::Mix)), "Mix");


        // b2ParticleDef
        m->add(user_type<b2ParticleDef>(), "b2ParticleDef");
        m->add(constructor<b2ParticleDef()>(), "b2ParticleDef");
        m->add(fun(&b2ParticleDef::flags), "flags");
        m->add(fun(&b2ParticleDef::position), "position");
        m->add(fun(&b2ParticleDef::velocity), "velocity");
        m->add(fun(&b2ParticleDef::color), "color");
        m->add(fun(&b2ParticleDef::lifetime), "lifetime");
        //m->add(fun(&b2ParticleDef::userData), "userData");
        m->add(fun(&b2ParticleDef::group), "group");

        return m;
      }

      /**
       * Register a ChaiScript module with b2ParticleGroup
       */
      ModulePtr addb2ParticleGroup(ModulePtr m = std::make_shared<Module>())
      {
        // b2ParticleGroupFlag
        m->add(chaiscript::type_conversion<b2ParticleGroupFlag, int>());
        m->add_global_const(const_var(b2_solidParticleGroup), "b2_solidParticleGroup");
        m->add_global_const(const_var(b2_rigidParticleGroup), "b2_rigidParticleGroup");
        m->add_global_const(const_var(b2_particleGroupCanBeEmpty), "b2_particleGroupCanBeEmpty");
        m->add_global_const(const_var(b2_particleGroupWillBeDestroyed), "b2_particleGroupWillBeDestroyed");
        m->add_global_const(const_var(b2_particleGroupNeedsUpdateDepth), "b2_particleGroupNeedsUpdateDepth");
        m->add_global_const(const_var(b2_particleGroupInternalMask), "b2_particleGroupInternalMask");
        m->add(chaiscript::type_conversion<const b2ParticleGroupFlag, b2ParticleGroupFlag>());
        m->add(fun([](b2ParticleGroupFlag& left, const b2ParticleGroupFlag& right) -> b2ParticleGroupFlag& {
          return (left = right);
        }), "=");

        // b2ParticleGroupDef
        m->add(user_type<b2ParticleGroupDef>(), "b2ParticleGroupDef");
        m->add(constructor<b2ParticleGroupDef()>(), "b2ParticleGroupDef");
        m->add(fun(&b2ParticleGroupDef::flags), "flags");
        m->add(fun(&b2ParticleGroupDef::position), "position");
        m->add(fun(&b2ParticleGroupDef::angle), "angle");
        m->add(fun(&b2ParticleGroupDef::linearVelocity), "linearVelocity");
        m->add(fun(&b2ParticleGroupDef::angularVelocity), "angularVelocity");
        m->add(fun(&b2ParticleGroupDef::color), "color");
        m->add(fun(&b2ParticleGroupDef::strength), "strength");
        m->add(fun(&b2ParticleGroupDef::shape), "shape");
        m->add(fun(&b2ParticleGroupDef::shapes), "shapes");
        m->add(fun(&b2ParticleGroupDef::shapeCount), "shapeCount");
        m->add(fun(&b2ParticleGroupDef::stride), "stride");
        m->add(fun(&b2ParticleGroupDef::particleCount), "particleCount");
        m->add(fun(&b2ParticleGroupDef::positionData), "positionData");
        m->add(fun(&b2ParticleGroupDef::lifetime), "lifetime");
        //m->add(fun(&b2ParticleGroupDef::userData), "userData");
        m->add(fun(&b2ParticleGroupDef::group), "group");
        m->add(fun([](b2ParticleGroupDef* f, const b2Shape* s) {
          f->shape = s;
        }), "SetShape");
        m->add(fun([](b2ParticleGroupDef* f, const b2Shape* const* s) {
          f->shapes = s;
        }), "SetShapes");

        // b2ParticleGroup
        m->add(user_type<b2ParticleGroup>(), "b2ParticleGroup");
        m->add(fun(static_cast<b2ParticleGroup*( b2ParticleGroup::*)()>(&b2ParticleGroup::GetNext)), "GetNext");
        m->add(fun(static_cast<const b2ParticleGroup*( b2ParticleGroup::*)() const>(&b2ParticleGroup::GetNext)), "GetNext");
        m->add(fun(static_cast<b2ParticleSystem*(b2ParticleGroup::*)()>(&b2ParticleGroup::GetParticleSystem)), "GetParticleSystem");
        m->add(fun(static_cast<const b2ParticleSystem*(b2ParticleGroup::*)() const>(&b2ParticleGroup::GetParticleSystem)), "GetParticleSystem");
        m->add(fun(&b2ParticleGroup::GetParticleCount), "GetParticleCount");
        m->add(fun(&b2ParticleGroup::GetBufferIndex), "GetBufferIndex");
        m->add(fun(&b2ParticleGroup::ContainsParticle), "ContainsParticle");
        m->add(fun(&b2ParticleGroup::GetAllParticleFlags), "GetAllParticleFlags");
        m->add(fun(&b2ParticleGroup::GetGroupFlags), "GetGroupFlags");
        m->add(fun(&b2ParticleGroup::SetGroupFlags), "SetGroupFlags");
        m->add(fun(&b2ParticleGroup::GetMass), "GetMass");
        m->add(fun(&b2ParticleGroup::GetInertia), "GetInertia");
        m->add(fun(&b2ParticleGroup::GetCenter), "GetCenter");
        m->add(fun(&b2ParticleGroup::GetLinearVelocity), "GetLinearVelocity");
        m->add(fun(&b2ParticleGroup::GetAngularVelocity), "GetAngularVelocity");
        m->add(fun(&b2ParticleGroup::GetTransform), "GetTransform");
        m->add(fun(&b2ParticleGroup::GetPosition), "GetPosition");
        m->add(fun(&b2ParticleGroup::GetAngle), "GetAngle");
        m->add(fun(&b2ParticleGroup::GetLinearVelocityFromWorldPoint), "GetLinearVelocityFromWorldPoint");
        //m->add(fun(&b2ParticleGroup::GetUserData), "GetUserData");
        //m->add(fun(&b2ParticleGroup::SetUserData), "SetUserData");
        m->add(fun(&b2ParticleGroup::ApplyForce), "ApplyForce");
        m->add(fun(&b2ParticleGroup::ApplyLinearImpulse), "ApplyLinearImpulse");
        m->add(fun(static_cast<void( b2ParticleGroup::*)(bool)>(&b2ParticleGroup::DestroyParticles)), "DestroyParticles");
        m->add(fun(static_cast<void( b2ParticleGroup::*)()>(&b2ParticleGroup::DestroyParticles)), "DestroyParticles");

        return m;
      }

      /**
       * Register a ChaiScript module with b2ParticleSystem
       */
      ModulePtr addb2ParticleSystem(ModulePtr m = std::make_shared<Module>())
      {
        // b2ParticleContact
        m->add(user_type<b2ParticleContact>(), "b2ParticleContact");
        m->add(constructor<b2ParticleContact()>(), "b2ParticleContact");
        m->add(fun(&b2ParticleContact::SetIndices), "SetIndices");
        m->add(fun(&b2ParticleContact::SetWeight), "SetWeight");
        m->add(fun(&b2ParticleContact::SetNormal), "SetNormal");
        m->add(fun(&b2ParticleContact::SetFlags), "SetFlags");
        m->add(fun(&b2ParticleContact::GetIndexA), "GetIndexA");
        m->add(fun(&b2ParticleContact::GetIndexB), "GetIndexB");
        m->add(fun(&b2ParticleContact::GetWeight), "GetWeight");
        m->add(fun(&b2ParticleContact::GetNormal), "GetNormal");
        m->add(fun(&b2ParticleContact::GetFlags), "GetFlags");
        m->add(fun(&b2ParticleContact::ApproximatelyEqual), "ApproximatelyEqual");
        m->add(fun([](const b2ParticleContact& lhs, const b2ParticleContact& rhs)
        {
          return  lhs == rhs;
        }), "==");

        // b2ParticleBodyContact
        m->add(user_type<b2ParticleBodyContact>(), "b2ParticleBodyContact");
        m->add(constructor<b2ParticleBodyContact()>(), "b2ParticleBodyContact");
        m->add(fun(&b2ParticleBodyContact::index), "index");
        m->add(fun(&b2ParticleBodyContact::body), "body");
        m->add(fun(&b2ParticleBodyContact::fixture), "fixture");
        m->add(fun(&b2ParticleBodyContact::weight), "weight");
        m->add(fun(&b2ParticleBodyContact::normal), "normal");
        m->add(fun(&b2ParticleBodyContact::mass), "mass");
        m->add(fun([](b2ParticleBodyContact& lhs, const b2ParticleBodyContact& rhs)
        {
          return  lhs = rhs;
        }), "=");

        // b2ParticlePair
        m->add(user_type<b2ParticlePair>(), "b2ParticlePair");
        m->add(constructor<b2ParticlePair()>(), "b2ParticlePair");
        m->add(fun(&b2ParticlePair::indexA), "indexA");
        m->add(fun(&b2ParticlePair::indexB), "indexB");
        m->add(fun(&b2ParticlePair::flags), "flags");
        m->add(fun(&b2ParticlePair::strength), "strength");
        m->add(fun(&b2ParticlePair::distance), "distance");
        m->add(fun([](b2ParticlePair& lhs, const b2ParticlePair& rhs)
        {
          return  lhs = rhs;
        }), "=");

        // b2ParticleTriad
        m->add(user_type<b2ParticleTriad>(), "b2ParticleTriad");
        m->add(constructor<b2ParticleTriad()>(), "b2ParticleTriad");
        m->add(fun(&b2ParticleTriad::indexA), "indexA");
        m->add(fun(&b2ParticleTriad::indexB), "indexB");
        m->add(fun(&b2ParticleTriad::flags), "flags");
        m->add(fun(&b2ParticleTriad::strength), "strength");
        m->add(fun(&b2ParticleTriad::pa), "pa");
        m->add(fun(&b2ParticleTriad::pb), "pb");
        m->add(fun(&b2ParticleTriad::pc), "pc");
        m->add(fun(&b2ParticleTriad::ka), "ka");
        m->add(fun(&b2ParticleTriad::kb), "kb");
        m->add(fun(&b2ParticleTriad::kc), "kc");
        m->add(fun(&b2ParticleTriad::s), "s");
        m->add(fun([](b2ParticleTriad& lhs, const b2ParticleTriad& rhs)
        {
          return  lhs = rhs;
        }), "=");

        // b2ParticleSystemDef
        m->add(user_type<b2ParticleSystemDef>(), "b2ParticleSystemDef");
        m->add(constructor<b2ParticleSystemDef()>(), "b2ParticleSystemDef");
        m->add(fun(&b2ParticleSystemDef::strictContactCheck), "strictContactCheck");
        m->add(fun(&b2ParticleSystemDef::density), "density");
        m->add(fun(&b2ParticleSystemDef::gravityScale), "gravityScale");
        m->add(fun(&b2ParticleSystemDef::radius), "radius");
        m->add(fun(&b2ParticleSystemDef::maxCount), "maxCount");
        m->add(fun(&b2ParticleSystemDef::pressureStrength), "pressureStrength");
        m->add(fun(&b2ParticleSystemDef::dampingStrength), "dampingStrength");
        m->add(fun(&b2ParticleSystemDef::elasticStrength), "elasticStrength");
        m->add(fun(&b2ParticleSystemDef::springStrength), "springStrength");
        m->add(fun(&b2ParticleSystemDef::viscousStrength), "viscousStrength");
        m->add(fun(&b2ParticleSystemDef::surfaceTensionPressureStrength), "surfaceTensionPressureStrength");
        m->add(fun(&b2ParticleSystemDef::surfaceTensionNormalStrength), "surfaceTensionNormalStrength");
        m->add(fun(&b2ParticleSystemDef::repulsiveStrength), "repulsiveStrength");
        m->add(fun(&b2ParticleSystemDef::powderStrength), "powderStrength");
        m->add(fun(&b2ParticleSystemDef::ejectionStrength), "ejectionStrength");
        m->add(fun(&b2ParticleSystemDef::staticPressureStrength), "staticPressureStrength");
        m->add(fun(&b2ParticleSystemDef::staticPressureRelaxation), "staticPressureRelaxation");
        m->add(fun(&b2ParticleSystemDef::staticPressureIterations), "staticPressureIterations");
        m->add(fun(&b2ParticleSystemDef::colorMixingStrength), "colorMixingStrength");
        m->add(fun(&b2ParticleSystemDef::destroyByAge), "destroyByAge");
        m->add(fun(&b2ParticleSystemDef::lifetimeGranularity), "lifetimeGranularity");

        // b2ParticleSystem
        m->add(user_type<b2ParticleSystem>(), "b2ParticleSystem");
        m->add(fun(&b2ParticleSystem::CreateParticle), "CreateParticle");
        m->add(fun(&b2ParticleSystem::GetParticleHandleFromIndex), "GetParticleHandleFromIndex");
        m->add(fun(static_cast<void( b2ParticleSystem::*)(int)>(&b2ParticleSystem::DestroyParticle)), "DestroyParticle");
        m->add(fun(static_cast<void( b2ParticleSystem::*)(int, bool)>(&b2ParticleSystem::DestroyParticle)), "DestroyParticle");
        m->add(fun(&b2ParticleSystem::DestroyOldestParticle), "DestroyOldestParticle");
        m->add(fun(static_cast<int32( b2ParticleSystem::*)(const b2Shape&, const b2Transform&)>(&b2ParticleSystem::DestroyParticlesInShape)), "DestroyParticlesInShape");
        m->add(fun(static_cast<int( b2ParticleSystem::*)(const b2Shape&, const b2Transform&, bool)>(&b2ParticleSystem::DestroyParticlesInShape)), "DestroyParticlesInShape");
        m->add(fun(&b2ParticleSystem::CreateParticleGroup), "CreateParticleGroup");
        m->add(fun(&b2ParticleSystem::JoinParticleGroups), "JoinParticleGroups");
        m->add(fun(&b2ParticleSystem::SplitParticleGroup), "SplitParticleGroup");
        m->add(fun(static_cast<b2ParticleGroup*(b2ParticleSystem::*)()>(&b2ParticleSystem::GetParticleGroupList)), "GetParticleGroupList");
        m->add(fun(static_cast<const b2ParticleGroup*(b2ParticleSystem::*)() const>(&b2ParticleSystem::GetParticleGroupList)), "GetParticleGroupList");
        m->add(fun(&b2ParticleSystem::GetParticleGroupCount), "GetParticleGroupCount");
        m->add(fun(&b2ParticleSystem::GetParticleCount), "GetParticleCount");
        m->add(fun(&b2ParticleSystem::GetMaxParticleCount), "GetMaxParticleCount");
        m->add(fun(&b2ParticleSystem::SetMaxParticleCount), "SetMaxParticleCount");
        m->add(fun(&b2ParticleSystem::GetAllParticleFlags), "GetAllParticleFlags");
        m->add(fun(&b2ParticleSystem::GetAllGroupFlags), "GetAllGroupFlags");
        m->add(fun(&b2ParticleSystem::SetPaused), "SetPaused");
        m->add(fun(&b2ParticleSystem::GetPaused), "GetPaused");
        m->add(fun(&b2ParticleSystem::SetDensity), "SetDensity");
        m->add(fun(&b2ParticleSystem::GetDensity), "GetDensity");
        m->add(fun(&b2ParticleSystem::SetGravityScale), "SetGravityScale");
        m->add(fun(&b2ParticleSystem::GetGravityScale), "GetGravityScale");
        m->add(fun(&b2ParticleSystem::SetDamping), "SetDamping");
        m->add(fun(&b2ParticleSystem::GetDamping), "GetDamping");
        m->add(fun(&b2ParticleSystem::SetStaticPressureIterations), "SetStaticPressureIterations");
        m->add(fun(&b2ParticleSystem::GetStaticPressureIterations), "GetStaticPressureIterations");
        m->add(fun(&b2ParticleSystem::SetRadius), "SetRadius");
        m->add(fun(&b2ParticleSystem::GetRadius), "GetRadius");
        m->add(fun(static_cast<b2Vec2*(b2ParticleSystem::*)()>(&b2ParticleSystem::GetPositionBuffer)), "GetPositionBuffer");
        m->add(fun(static_cast<const b2Vec2*(b2ParticleSystem::*)() const>(&b2ParticleSystem::GetPositionBuffer)), "GetPositionBuffer");
        m->add(fun(static_cast<b2ParticleColor*(b2ParticleSystem::*)()>(&b2ParticleSystem::GetColorBuffer)), "GetColorBuffer");
        m->add(fun(static_cast<const b2ParticleColor*(b2ParticleSystem::*)() const>(&b2ParticleSystem::GetColorBuffer)), "GetColorBuffer");
        m->add(fun(static_cast<b2ParticleGroup* const*(b2ParticleSystem::*)()>(&b2ParticleSystem::GetGroupBuffer)), "GetGroupBuffer");
        m->add(fun(static_cast<const b2ParticleGroup* const*(b2ParticleSystem::*)() const>(&b2ParticleSystem::GetGroupBuffer)), "GetGroupBuffer");
        m->add(fun(static_cast<float32*(b2ParticleSystem::*)()>(&b2ParticleSystem::GetWeightBuffer)), "GetWeightBuffer");
        m->add(fun(static_cast<const float32*(b2ParticleSystem::*)() const>(&b2ParticleSystem::GetWeightBuffer)), "GetWeightBuffer");
        //m->add(fun(static_cast<void**(b2ParticleSystem::*)()>(&b2ParticleSystem::GetUserDataBuffer)), "GetUserDataBuffer");
        //m->add(fun(static_cast<void* const*(b2ParticleSystem::*)() const>(&b2ParticleSystem::GetUserDataBuffer)), "GetUserDataBuffer");
        m->add(fun(&b2ParticleSystem::GetFlagsBuffer), "GetFlagsBuffer");
        m->add(fun(&b2ParticleSystem::SetParticleFlags), "SetParticleFlags");
        m->add(fun(&b2ParticleSystem::GetParticleFlags), "GetParticleFlags");
        m->add(fun(&b2ParticleSystem::SetFlagsBuffer), "SetFlagsBuffer");
        m->add(fun(&b2ParticleSystem::SetPositionBuffer), "SetPositionBuffer");
        m->add(fun(&b2ParticleSystem::SetVelocityBuffer), "SetVelocityBuffer");
        m->add(fun(&b2ParticleSystem::SetColorBuffer), "SetColorBuffer");
        //m->add(fun(&b2ParticleSystem::SetUserDataBuffer), "SetUserDataBuffer");
        m->add(fun(&b2ParticleSystem::GetContacts), "GetContacts");
        m->add(fun(&b2ParticleSystem::GetContactCount), "GetContactCount");
        m->add(fun(&b2ParticleSystem::GetBodyContacts), "GetBodyContacts");
        m->add(fun(&b2ParticleSystem::GetBodyContactCount), "GetBodyContactCount");
        m->add(fun(&b2ParticleSystem::GetPairs), "GetPairs");
        m->add(fun(&b2ParticleSystem::GetPairCount), "GetPairCount");
        m->add(fun(&b2ParticleSystem::GetTriads), "GetTriads");
        m->add(fun(&b2ParticleSystem::GetTriadCount), "GetTriadCount");
        m->add(fun(&b2ParticleSystem::SetStuckThreshold), "SetStuckThreshold");
        m->add(fun(&b2ParticleSystem::GetStuckCandidates), "GetStuckCandidates");
        m->add(fun(&b2ParticleSystem::GetStuckCandidateCount), "GetStuckCandidateCount");
        m->add(fun(&b2ParticleSystem::ComputeCollisionEnergy), "ComputeCollisionEnergy");
        m->add(fun(&b2ParticleSystem::SetStrictContactCheck), "SetStrictContactCheck");
        m->add(fun(&b2ParticleSystem::GetStrictContactCheck), "GetStrictContactCheck");
        m->add(fun(&b2ParticleSystem::SetParticleLifetime), "SetParticleLifetime");
        m->add(fun(&b2ParticleSystem::GetParticleLifetime), "GetParticleLifetime");
        m->add(fun(&b2ParticleSystem::SetDestructionByAge), "SetDestructionByAge");
        m->add(fun(&b2ParticleSystem::GetDestructionByAge), "GetDestructionByAge");
        m->add(fun(&b2ParticleSystem::GetExpirationTimeBuffer), "GetExpirationTimeBuffer");
        m->add(fun(&b2ParticleSystem::ExpirationTimeToLifetime), "ExpirationTimeToLifetime");
        m->add(fun(&b2ParticleSystem::GetIndexByExpirationTimeBuffer), "GetIndexByExpirationTimeBuffer");
        m->add(fun(&b2ParticleSystem::ParticleApplyLinearImpulse), "ParticleApplyLinearImpulse");
        m->add(fun(&b2ParticleSystem::ApplyLinearImpulse), "ApplyLinearImpulse");
        m->add(fun(&b2ParticleSystem::ParticleApplyForce), "ParticleApplyForce");
        m->add(fun(&b2ParticleSystem::ApplyForce), "ApplyForce");
        m->add(fun(static_cast<b2ParticleSystem*(b2ParticleSystem::*)()>(&b2ParticleSystem::GetNext)), "GetNext");
        m->add(fun(static_cast<const b2ParticleSystem*(b2ParticleSystem::*)() const>(&b2ParticleSystem::GetNext)), "GetNext");
        m->add(fun(&b2ParticleSystem::QueryAABB), "QueryAABB");
        m->add(fun(&b2ParticleSystem::QueryShapeAABB), "QueryShapeAABB");
        m->add(fun(&b2ParticleSystem::RayCast), "RayCast");
        m->add(fun(&b2ParticleSystem::ComputeAABB), "ComputeAABB");

        return m;
      }

      /**
       * Register a ChaiScript module with b2Math.
       */
      ModulePtr addb2Math(ModulePtr m = std::make_shared<Module>()) {
        // b2Vec2
        m->add(user_type<b2Vec2>(), "b2Vec2");
        m->add(constructor<b2Vec2()>(), "b2Vec2");
        m->add(constructor<b2Vec2(float32, float32)>(), "b2Vec2");
        m->add(fun(&b2Vec2::x), "x");
        m->add(fun(&b2Vec2::y), "y");
        m->add(fun(&b2Vec2::Set), "Set");
        m->add(fun(&b2Vec2::SetZero), "SetZero");
        m->add(fun(&b2Vec2::Length), "Length");
        m->add(fun(&b2Vec2::LengthSquared), "LengthSquared");
        m->add(fun(&b2Vec2::Normalize), "Normalize");
        m->add(fun(&b2Vec2::IsValid), "IsValid");
        m->add(fun(&b2Vec2::Skew), "Skew");

        // b2Vec3
        m->add(user_type<b2Vec3>(), "b2Vec3");
        m->add(constructor<b2Vec3()>(), "b2Vec3");
        m->add(constructor<b2Vec3(float32, float32, float32)>(), "b2Vec3");
        m->add(fun(&b2Vec3::x), "x");
        m->add(fun(&b2Vec3::y), "y");
        m->add(fun(&b2Vec3::z), "z");
        m->add(fun(&b2Vec3::Set), "Set");
        m->add(fun(&b2Vec3::SetZero), "SetZero");

        // b2Mat22
        m->add(user_type<b2Mat22>(), "b2Mat22");
        m->add(constructor<b2Mat22()>(), "b2Mat22");
        m->add(constructor<b2Mat22(float32, float32, float32, float32)>(), "b2Mat22");
        m->add(constructor<b2Mat22(const b2Vec2&, const b2Vec2&)>(), "b2Mat22");
        m->add(fun(&b2Mat22::ex), "ex");
        m->add(fun(&b2Mat22::ey), "ey");
        m->add(fun(&b2Mat22::Set), "Set");
        m->add(fun(&b2Mat22::SetIdentity), "SetIdentity");
        m->add(fun(&b2Mat22::SetZero), "SetZero");
        m->add(fun(&b2Mat22::GetInverse), "GetInverse");
        m->add(fun(&b2Mat22::Solve), "Solve");

        // b2Mat33
        m->add(user_type<b2Mat33>(), "b2Mat33");
        m->add(constructor<b2Mat33()>(), "b2Mat33");
        m->add(constructor<b2Mat33(const b2Vec3&, const b2Vec3&, const b2Vec3&)>(), "b2Mat33");
        m->add(fun(&b2Mat33::ex), "ex");
        m->add(fun(&b2Mat33::ey), "ey");
        m->add(fun(&b2Mat33::ez), "ez");
        m->add(fun(&b2Mat33::SetZero), "SetZero");
        m->add(fun(&b2Mat33::Solve33), "Solve33");
        m->add(fun(&b2Mat33::Solve22), "Solve22");
        m->add(fun(&b2Mat33::GetInverse22), "GetInverse22");
        m->add(fun(&b2Mat33::GetSymInverse33), "GetSymInverse33");

        // b2Rot
        m->add(user_type<b2Rot>(), "b2Rot");
        m->add(constructor<b2Rot()>(), "b2Rot");
        m->add(constructor<b2Rot(float32)>(), "b2Rot");
        m->add(fun(&b2Rot::s), "s");
        m->add(fun(&b2Rot::c), "c");
        m->add(fun(&b2Rot::SetIdentity), "SetIdentity");
        m->add(fun(&b2Rot::GetAngle), "GetAngle");
        m->add(fun(&b2Rot::GetXAxis), "GetXAxis");
        m->add(fun(&b2Rot::GetYAxis), "GetYAxis");

        // b2Transform
        m->add(user_type<b2Transform>(), "b2Transform");
        m->add(constructor<b2Transform()>(), "b2Transform");
        m->add(constructor<b2Transform(const b2Vec2&, const b2Rot&)>(), "b2Transform");
        m->add(fun(&b2Transform::p), "p");
        m->add(fun(&b2Transform::q), "q");
        m->add(fun(&b2Transform::SetIdentity), "SetIdentity");
        m->add(fun(&b2Transform::Set), "Set");

        // b2Sweep
        m->add(user_type<b2Sweep>(), "b2Sweep");
        m->add(fun(&b2Sweep::GetTransform), "GetTransform");
        m->add(fun(&b2Sweep::Advance), "Advance");
        m->add(fun(&b2Sweep::Normalize), "Normalize");

        return m;
      }

      /**
       * Bootstrap a ChaiScript module with Box2D support.
       */
      ModulePtr bootstrap(ModulePtr m = std::make_shared<Module>())
      {
        addb2BlockAllocator(m);
        addb2Draw(m);
        addb2Body(m);
        addb2StackAllocator(m);
        addb2PolygonShape(m);
        addb2Fixture(m);
        addb2World(m);
        addb2Math(m);
        addb2Timer(m);
        addb2ContactManager(m);
        addb2Particle(m);
        addb2ParticleGroup(m);
        addb2ParticleSystem(m);

        return m;
      }
    }
  }
}
