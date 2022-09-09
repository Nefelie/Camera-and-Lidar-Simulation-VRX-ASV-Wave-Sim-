/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <ignition/msgs/wrench.pb.h>
#include <chrono>
#include <string>
#include <unordered_map>
#include <ignition/common/Profiler.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/World.hh"

#include "Surface.hh"
#include "Wavefield.hh"

using namespace ignition;
using namespace vrx;

/// \brief Private Surface data class.
class vrx::Surface::Implementation
{
  /// \brief Parse the points via SDF.
  /// \param[in] _sdf Pointer to the SDF.
  public: void ParsePoints(const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief The link entity.
  public: gazebo::Link link;

  /// \brief Vessel length [m].
  public: double hullLength = 4.9;

  /// \brief Demi-hull radius [m].
  public: double hullRadius = 0.213;

  /// \brief Fluid height [m].
  public: double fluidLevel = 0;

  /// \brief Fluid density [kg/m^3].
  public: double fluidDensity = 997.7735;

  /// \brief The world's gravity [m/s^2].
  public: math::Vector3d gravity;

  /// \brief The points where the plugin applies forces. These points are
  /// relative to the link paramter's origin. Note that we don't check that the
  /// points are contained within the hull. You should pass reasonable points.
  public: std::vector<math::Vector3d> points;

  /// \brief The wavefield.
  public: Wavefield wavefield;
};

//////////////////////////////////////////////////
void Surface::Implementation::ParsePoints(
  const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (!_sdf->HasElement("points"))
    return;

  auto ptr = const_cast<sdf::Element *>(_sdf.get());
  auto sdfPoints = ptr->GetElement("points");

  // We need at least one point.
  if (!sdfPoints->HasElement("point"))
    ignerr << "Unable to find <points><point> element in SDF." << std::endl;

  auto pointElem = sdfPoints->GetElement("point");

  // Parse a new point.
  while (pointElem)
  {
    ignition::math::Vector3d point;
    pointElem->GetValue()->Get<math::Vector3d>(point);
    this->points.push_back(point);

    // Parse the next point.
    pointElem = pointElem->GetNextElement("point");
  }





  // if (!_sdf->HasElement("hull"))
  //  return;

  // auto ptr = const_cast<sdf::Element *>(_sdf.get());


  // auto sdfHull = ptr->GetElement("hull");
  // while (sdfHull)
  // {
  //   // We need the hull's name.
  //   if (!sdfHull->HasElement("name"))
  //     ignerr << "Unable to find <hull><name> element in SDF." << std::endl;

  //   std::string name = sdfHull->Get<std::string>("name");

  //   // We need at least one point.
  //   if (!sdfHull->HasElement("point"))
  //     ignerr << "Unable to find <hull><point> element in SDF." << std::endl;

  //   auto pointElem = sdfHull->GetElement("point");
  //   while (pointElem)
  //   {
  //     ignition::math::Vector3d point;
  //     pointElem->GetValue()->Get<math::Vector3d>(point);
  //     this->hulls[name].push_back(point);

  //     // Parse the next point.
  //     pointElem = pointElem->GetNextElement("point");
  //   }

  //   sdfHull = sdfHull->GetNextElement("hull");
  // }
}

//////////////////////////////////////////////////
double Surface::CircleSegment(double _r, double _h) const
{
  return _r * _r * acos((_r -_h) / _r ) -
    (_r - _h) * sqrt(2 * _r * _h - _h * _h);
}

//////////////////////////////////////////////////
Surface::Surface()
  : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void Surface::Configure(const gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gazebo::EntityComponentManager &_ecm,
    gazebo::EventManager &/*_eventMgr*/)
{
  // Parse required elements.
  if (!_sdf->HasElement("link_name"))
  {
    ignerr << "No <link_name> specified" << std::endl;
    return;
  }

  gazebo::Model model(_entity);
  std::string linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->link = gazebo::Link(model.LinkByName(_ecm, linkName));
  if (!this->dataPtr->link.Valid(_ecm))
  {
    ignerr << "Could not find link named [" << linkName
           << "] in model" << std::endl;
    return;
  }

  // Optional parameters.
  // Although some of these parameters are required in this plugin, a potential
  // derived plugin might not need them. Make sure that the default values are
  // reasonable.
  if (_sdf->HasElement("hull_length"))
  {
    this->dataPtr->hullLength = _sdf->Get<double>("hull_length");
  }

  if (_sdf->HasElement("hull_radius"))
  {
    this->dataPtr->hullRadius = _sdf->Get<double>("hull_radius");
  }

  if (_sdf->HasElement("fluid_level"))
  {
    this->dataPtr->fluidLevel = _sdf->Get<double>("fluid_level");
  }

  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  // Parse the optional <points> element.
  this->dataPtr->ParsePoints(_sdf);

  // Get the gravity from the world.
  auto worldEntity = gazebo::worldEntity(_ecm);
  auto world = gazebo::World(worldEntity);
  auto gravityOpt = world.Gravity(_ecm);
  if (!gravityOpt)
  {
    ignerr << "Unable to get the gravity from the world" << std::endl;
    return;
  }
  this->dataPtr->gravity = *gravityOpt;

  // Wavefield
  this->dataPtr->wavefield.Load(_sdf);

  igndbg << "Surface plugin successfully configured with the following "
         << "parameters:" << std::endl;
  igndbg << "  <link_name>: " << linkName << std::endl;
  igndbg << "  <vehicle_length>: " << this->dataPtr->hullLength << std::endl;
  igndbg << "  <hull_radius>: " << this->dataPtr->hullRadius << std::endl;
  igndbg << "  <fluid_level>: " << this->dataPtr->fluidLevel << std::endl;
  igndbg << "  <fluid_density>: " << this->dataPtr->fluidDensity << std::endl;
  igndbg << "  <points>:" << std::endl;
  for (const auto &p : this->dataPtr->points)
    igndbg << "    [" << p << "]" << std::endl;
}

//////////////////////////////////////////////////
void Surface::PreUpdate(const gazebo::UpdateInfo &_info,
    gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("Surface::PreUpdate");

  if (_info.paused)
    return;

  // Vehicle frame transform.
  const auto kPose = this->dataPtr->link.WorldPose(_ecm);
  if (!kPose)
  {
    ignerr << "Unable to get world pose from link ["
           << this->dataPtr->link.Entity() << "]" << std::endl;
    return;
  }
  const math::Vector3d kEuler = (*kPose).Rot().Euler();
  math::Quaternion vq(kEuler.X(), kEuler.Y(), kEuler.Z());

  for (auto const &bpnt : this->dataPtr->points)
  {
    // Transform from vessel to fluid/world frame.
    const math::Vector3d kBpntW = vq * bpnt;

    // Vertical location of boat grid point in world frame.
    const float kDdz = (*kPose).Pos().Z() + kBpntW.Z();

    // World location of grid point.
    math::Vector3d point;
    point.X() = (*kPose).Pos().X() + kBpntW.X();
    point.Y() = (*kPose).Pos().Y() + kBpntW.Y();

    // Compute the depth at the grid point.
    double simTime = std::chrono::duration<double>(_info.simTime).count();
    double depth = this->dataPtr->wavefield.ComputeDepthSimply(point, simTime);

    // Vertical wave displacement.
    double dz = depth + point.Z();

    // Total z location of boat grid point relative to fluid surface.
    double deltaZ = (this->dataPtr->fluidLevel + dz) - kDdz;
    // Enforce only upward buoy force
    deltaZ = std::max(deltaZ, 0.0);
    deltaZ = std::min(deltaZ, this->dataPtr->hullRadius);

    const float kBuoyForce =
      this->CircleSegment(this->dataPtr->hullRadius, deltaZ) *
        this->dataPtr->hullLength / this->dataPtr->points.size() *
          -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity;

    // Apply force at the point.
    // Position is in the link frame and force is in world frame.
    this->dataPtr->link.AddWorldForce(_ecm,
      math::Vector3d(0, 0, kBuoyForce),
      bpnt);

    // Debug output:
    // igndbg << bpnt.X() << "," << bpnt.Y() << "," << bpnt.Z() << std::endl;
    // igndbg << "-" << std::endl;
    // igndbg << bpntW.X() << "," << bpntW.Y() << "," << bpntW.Z() << std::endl;
    // igndbg << "X: " << X << std::endl;
    // igndbg << "depth: " << depth << std::endl;
    // igndbg << "dz: " << dz << std::endl;
    // igndbg << "kDdz: " << kDdz << std::endl;
    // igndbg << "deltaZ: " << deltaZ << std::endl;
    // igndbg << "hull radius: " << this->dataPtr->hullRadius << std::endl;
    // igndbg << "vehicle length: " << this->dataPtr->hullLength << std::endl;
    // igndbg << "gravity z: " << -this->dataPtr->gravity.Z() << std::endl;
    // igndbg << "fluid density: " << this->dataPtr->fluidDensity << std::endl;
    // igndbg << "Force: " << kBuoyForce << std::endl << std::endl;
  }
}

//////////////////////////////////////////////////
math::Vector3d Surface::Gravity() const
{
  return this->dataPtr->gravity;
}

//////////////////////////////////////////////////
double Surface::HullLength() const
{
  return this->dataPtr->hullLength;
}

//////////////////////////////////////////////////
double Surface::HullRadius() const
{
  return this->dataPtr->hullRadius;
}

//////////////////////////////////////////////////
double Surface::FluidDensity() const
{
  return this->dataPtr->fluidDensity;
}

//////////////////////////////////////////////////
// double Surface::BuoyancyAtPoint(const gazebo::UpdateInfo &/*_info*/,
//     const math::Vector3d &/*_point*/, const uint16_t _pointsPerHull,
//     double _deltaZ,
//     gazebo::EntityComponentManager &/*_ecm*/)
// {
//   // Buoyancy force at this point.
//   return this->CircleSegment(this->dataPtr->hullRadius, _deltaZ) *
//     this->dataPtr->hullLength / _pointsPerHull *
//     -this->dataPtr->gravity.Z() * this->dataPtr->fluidDensity;
// }

IGNITION_ADD_PLUGIN(Surface,
                    gazebo::System,
                    Surface::ISystemConfigure,
                    Surface::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(vrx::Surface,
                          "vrx::Surface")