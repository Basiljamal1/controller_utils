
#include "controller_utils/kinematic_interfaces/JointReferenceInterface.hpp"

namespace controller_utils::interfaces {

JointReferenceInterface::JointReferenceInterface(
       size_t dof)  // Assuming 'dof' is the degree of freedom or size of
                    // vectors
         : mPositions(dof, std::numeric_limits<double>::quiet_NaN()),
           mVelocities(dof, std::numeric_limits<double>::quiet_NaN()),
           mTorques(dof, std::numeric_limits<double>::quiet_NaN()) {
}

void JointReferenceInterface::setPosition(const std::vector<double>& position) {
   if (position.size() != mPositions.size()) {
      spdlog::error(
             "The size of the position vector does not match the size of the "
             "reference position vector. Expected: {}, Received: {}",
             mPositions.size(), position.size());
      return;
   }
   std::copy(position.begin(), position.end(), mPositions.begin());
}

void JointReferenceInterface::setVelocity(const std::vector<double>& velocity) {
   if (velocity.size() != mVelocities.size()) {
      spdlog::error(
             "The size of the velocity vector does not match the size of the "
             "reference velocity vector. Expected: {}, Received: {}",
             mVelocities.size(), velocity.size());
      return;
   }
   std::copy(velocity.begin(), velocity.end(), mVelocities.begin());
}

void JointReferenceInterface::setTorque(const std::vector<double>& torque) {
   if (torque.size() != mTorques.size()) {
      spdlog::error(
             "The size of the torque vector does not match the size of the "
             "reference "
             "torque vector. Expected: {}, Received: {}",
             mTorques.size(), torque.size());
      return;
   }
   std::copy(torque.begin(), torque.end(), mTorques.begin());
}

const std::vector<double>& JointReferenceInterface::getReferencePositions()
       const {
   return mPositions;
}

const std::vector<double>& JointReferenceInterface::getReferenceVelocities()
       const {
   return mVelocities;
}

const std::vector<double>& JointReferenceInterface::getReferenceTorques()
       const {
   return mTorques;
}

double* JointReferenceInterface::exportReferencePosition(size_t index) {
   if (index >= mPositions.size()) {
      spdlog::error("Index out of bounds");
      return nullptr;
   }
   return &mPositions[index];
}

double* JointReferenceInterface::exportReferenceVelocity(size_t index) {
   if (index >= mVelocities.size()) {
      spdlog::error("Index out of bounds");
      return nullptr;
   }
   return &mVelocities[index];
}

double* JointReferenceInterface::exportReferenceTorque(size_t index) {
   if (index >= mTorques.size()) {
      spdlog::error("Index out of bounds");
      return nullptr;
   }
   return &mTorques[index];
}

size_t JointReferenceInterface::getDof() const {
   if (mPositions.size() > 0) {
      return mPositions.size();
   } else if (mVelocities.size() > 0) {
      return mVelocities.size();
   } else if (mTorques.size() > 0) {
      return mTorques.size();
   }

   return 0;
}

};  // namespace controller_utils::interfaces
