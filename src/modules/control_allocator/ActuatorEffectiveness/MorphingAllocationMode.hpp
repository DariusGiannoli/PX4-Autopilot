// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "MorphingQuadGeometry.hpp"

// Experimental selection changes only while disarmed. Arm prediction remains live.
class MorphingAllocationMode {
public:
 bool update(bool requested_frozen, bool armed) {
  if (armed || requested_frozen == _frozen) { return false; }
  _frozen = requested_frozen; return true;
 }
 morphing_quad::RotorValues select(const morphing_quad::RotorValues &predicted) const {
  return _frozen ? morphing_quad::RotorValues{} : predicted;
 }
private:
 bool _frozen{false};
};
