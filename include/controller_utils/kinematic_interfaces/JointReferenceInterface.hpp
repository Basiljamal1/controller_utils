#ifndef CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICREFERENCECOMMANDS_HPP_
#define CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICREFERENCECOMMANDS_HPP_

#include <cstddef>
#include <vector>
#include <spdlog/spdlog.h>

namespace controller_utils::interfaces {

constexpr size_t DEFAULT_DOF = 6;
/**
 * @class JointReferenceInterface
 * @brief Lightweight storage for kinematic reference commands.
 *
 * Encapsulates reference values (position, velocity, torque) for a multi-DOF
 * kinematic element. The interface offers simple getters/setters, per-index
 * exporters (raw pointers), and a query for the configured DOF.
 */
class JointReferenceInterface {
   public:
   /**
    * @brief Create a JointReferenceInterface with a given number of degrees of
    * freedom.
    *
    * @param dof Number of joints / channels to allocate (defaults to
    * DEFAULT_DOF).
    */
   explicit JointReferenceInterface(size_t dof = DEFAULT_DOF);

   /**
    * @brief Return the configured number of degrees of freedom.
    *
    * @return The DOF (size) used by this instance.
    */
   size_t getDof() const;

   /**
    * @brief Access a const reference to all stored position references.
    *
    * @return Vector of positions (read-only view).
    */
   const std::vector<double>& getReferencePositions() const;

   /**
    * @brief Access a const reference to all stored velocity references.
    *
    * @return Vector of velocities (read-only view).
    */
   const std::vector<double>& getReferenceVelocities() const;

   /**
    * @brief Access a const reference to all stored torque references.
    *
    * @return Vector of torques (read-only view).
    */
   const std::vector<double>& getReferenceTorques() const;

   /**
    * @brief Overwrite all stored position references.
    *
    * The input vector must match the configured DOF.
    *
    * @param position Vector of new position references.
    */
   void setPosition(const std::vector<double>& position);

   /**
    * @brief Overwrite all stored velocity references.
    *
    * The input vector must match the configured DOF.
    *
    * @param velocity Vector of new velocity references.
    */
   void setVelocity(const std::vector<double>& velocity);

   /**
    * @brief Overwrite all stored torque references.
    *
    * The input vector must match the configured DOF.
    *
    * @param torque Vector of new torque references.
    */
   void setTorque(const std::vector<double>& torque);

   /**
    * @brief Export a raw pointer to a stored position element.
    *
    * Returns a pointer to the internal storage for direct modification. The
    * caller is responsible for bounds correctness.
    *
    * @param index Element index in [0, getDof()).
    * @return Pointer to the requested position element, or nullptr if out of
    * range.
    */
   double* exportReferencePosition(size_t index);

   /**
    * @brief Export a raw pointer to a stored velocity element.
    *
    * @param index Element index in [0, getDof()).
    * @return Pointer to the requested velocity element, or nullptr if out of
    * range.
    */
   double* exportReferenceVelocity(size_t index);

   /**
    * @brief Export a raw pointer to a stored torque element.
    *
    * @param index Element index in [0, getDof()).
    * @return Pointer to the requested torque element, or nullptr if out of
    * range.
    */
   double* exportReferenceTorque(size_t index);

   private:
   std::vector<double>
          mPositions; /**< Internal storage for position references. */
   std::vector<double>
          mVelocities; /**< Internal storage for velocity references. */
   std::vector<double> mTorques; /**< Internal storage for torque references. */
};

}  // namespace controller_utils::interfaces

#endif  // CONTROLLER_UTILS___KINEMATIC_INTERFACES__KINEMATICREFERENCECOMMANDS_HPP_
