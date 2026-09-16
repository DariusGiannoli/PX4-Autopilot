// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "ActuatorEffectivenessMultirotor.hpp"
#include "MorphingQuadGeometry.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/morphing_geometry.h>
#include <uORB/topics/morphing_allocation_matrix.h>
#include <uORB/topics/morphing_arm_state.h>

/** Four horizontal arms. Consumes accepted angles published by external servo-driving code. */
class ActuatorEffectivenessMorphingQuad : public ActuatorEffectivenessMultirotor
{
public:
	explicit ActuatorEffectivenessMorphingQuad(ModuleParams *parent) : ActuatorEffectivenessMultirotor(parent) {}

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason reason) override;
	bool getNormalizationReference(int matrix_index, EffectivenessMatrix &reference) const override;
	void effectivenessMatrixApplied(int matrix_index, const EffectivenessMatrix &matrix, uint16_t failure_mask) override;
	uint32_t effectivenessUpdateIntervalUs() const override { return 10000; } // at most 100 Hz
	float minimumControlAuthority() const override { return 1e-6f; }
	bool useRowScaledInverse() const override { return true; }
	const char *name() const override { return "Morphing quadrotor"; }

private:
	bool computeMatrix(const morphing_quad::RotorValues &angles, EffectivenessMatrix &matrix, morphing_quad::GeometryResult *geometry_result = nullptr) const;
	void publishGeometry(bool matrix_updated);
	morphing_geometry_s _diagnostics{};
	uORB::Publication<morphing_geometry_s> _geometry_pub{ORB_ID(morphing_geometry)};
	uORB::Publication<morphing_allocation_matrix_s> _matrix_pub{ORB_ID(morphing_allocation_matrix)};
	uORB::Subscription _arm_state_sub{ORB_ID(morphing_arm_state)};
	morphing_quad::RotorValues _angles{}; // neutral until the first valid accepted state
	uint64_t _last_timestamp{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_MORPH_COGX>) _param_cog_x,
		(ParamFloat<px4::params::CA_MORPH_COGY>) _param_cog_y,
		(ParamFloat<px4::params::CA_MORPH_COGZ>) _param_cog_z
	)
};
