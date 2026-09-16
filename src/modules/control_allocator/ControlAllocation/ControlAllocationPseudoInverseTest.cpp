/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocationTest.cpp
 *
 * Tests for Control Allocation Algorithms
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include <gtest/gtest.h>
#include <ControlAllocationPseudoInverse.hpp>

using namespace matrix;

TEST(ControlAllocationTest, AllZeroCase)
{
	ControlAllocationPseudoInverse method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}

namespace
{
class TestPseudoInverse : public ControlAllocationPseudoInverse
{
public:
	const Matrix<float, 16, 6> &mix() const { return _mix; }
	const Vector<float, 6> &scale() const { return _control_allocation_scale; }
};

Matrix<float, 6, 16> physicalQuadMatrix()
{
	Matrix<float, 6, 16> b;
	const float active[4][4] {
		{-0.04f, 0.09f, 0.08f, -0.03f},
		{0.09f, -0.04f, 0.03f, -0.08f},
		{0.006f, 0.006f, -0.006f, -0.006f},
		{-1.f, -1.f, -1.f, -1.f}
	};

	for (int row = 0; row < 4; ++row) {
		for (int motor = 0; motor < 4; ++motor) { b(row == 3 ? 5 : row, motor) = active[row][motor]; }
	}

	return b;
}

void expectLegacyInverse(const TestPseudoInverse &allocator, const Matrix<float, 6, 16> &b, int count)
{
	Matrix<float, 16, 6> expected;
	matrix::geninv(b, expected);

	for (int motor = 0; motor < 16; ++motor) {
		for (int axis = 0; axis < 6; ++axis) {
			const float value = motor < count && fabsf(expected(motor, axis)) < 1e-3f ? 0.f : expected(motor, axis);
			EXPECT_FLOAT_EQ(allocator.mix()(motor, axis), value);
		}
	}
}
} // namespace

TEST(ControlAllocationTest, RowScalingRestoresPhysicalUnitsAndZeroAxes)
{
	TestPseudoInverse allocator;
	allocator.setUseRowScaledInverse(true);
	const auto b = physicalQuadMatrix();
	ControlAllocation::ActuatorVector trim;
	allocator.setEffectivenessMatrix(b, trim, trim, 4, false);
	allocator.allocate();
	const Matrix<float, 6, 6> product = b * allocator.mix();

	for (int row = 0; row < 6; ++row) {
		for (int col = 0; col < 6; ++col) {
			EXPECT_NEAR(product(row, col), row == col && row != 3 && row != 4 ? 1.f : 0.f, 1e-4f);
		}
	}

	for (int motor = 0; motor < 16; ++motor) {
		for (int axis = 0; axis < 6; ++axis) {
			if (motor >= 4 || axis == 3 || axis == 4) { EXPECT_FLOAT_EQ(allocator.mix()(motor, axis), 0.f); }
		}
	}
}

TEST(ControlAllocationTest, DefaultAndDisabledRowScalingUseLegacyInverse)
{
	TestPseudoInverse allocator;
	const auto b = physicalQuadMatrix();
	ControlAllocation::ActuatorVector trim;
	allocator.setEffectivenessMatrix(b, trim, trim, 4, false);
	allocator.allocate();
	expectLegacyInverse(allocator, b, 4);
	const auto legacy = allocator.mix();
	allocator.setUseRowScaledInverse(true);
	allocator.allocate(); // Changing policy must invalidate a previously cached inverse.
	EXPECT_GT((allocator.mix() - legacy).abs().max(), 1e-4f);
	allocator.setUseRowScaledInverse(false);
	allocator.allocate();
	expectLegacyInverse(allocator, b, 4);
}

TEST(ControlAllocationTest, RowScalingRetainsLegacyFallback)
{
	for (int scenario = 0; scenario < 12; ++scenario) {
		SCOPED_TRACE(scenario);
		TestPseudoInverse allocator;
		allocator.setUseRowScaledInverse(true);
		auto b = physicalQuadMatrix();
		int count = 4;

		if (scenario == 0) { allocator.setHadActuatorFailure(true); }
		if (scenario >= 1 && scenario <= 4) { b.col(scenario - 1) = 0.f; }
		if (scenario == 5) { b.row(2) = 0.f; }
		if (scenario == 6) { b.col(1) = b.col(0); }
		if (scenario == 7) { b.row(2) = b.row(0); }
		if (scenario == 8) { b.setZero(); }
		if (scenario == 9) { b(3, 0) = 0.02f; }
		if (scenario == 10) { b(5, 4) = -1.f; count = 5; }
		if (scenario == 11) { b(5, 4) = -1.f; } // Reject unexpected data outside the four motor columns.

		ControlAllocation::ActuatorVector trim;
		allocator.setEffectivenessMatrix(b, trim, trim, count, false);
		allocator.allocate();
		expectLegacyInverse(allocator, b, count);
	}
}

TEST(ControlAllocationTest, RowScalingUsesPhysicalNeutralNormalization)
{
	TestPseudoInverse allocator;
	allocator.setUseRowScaledInverse(true);
	allocator.setNormalizeRPY(true);
	auto reference = physicalQuadMatrix();
	const float x[4] {1.f, -1.f, 1.f, -1.f};
	const float y[4] {-1.f, 1.f, 1.f, -1.f};

	for (int motor = 0; motor < 4; ++motor) {
		reference(0, motor) = 0.07f * y[motor];
		reference(1, motor) = 0.07f * x[motor];
	}

	ControlAllocation::ActuatorVector trim;
	allocator.setEffectivenessMatrix(physicalQuadMatrix(), trim, trim, 4, true);
	allocator.setNormalizationReference(reference);
	EXPECT_NEAR(allocator.scale()(0), 1.f / (0.07f * sqrtf(8.f)), 1e-5f);
	EXPECT_NEAR(allocator.scale()(1), allocator.scale()(0), 1e-6f);
	EXPECT_NEAR(allocator.scale()(2), 1.f / (4.f * 0.006f), 1e-4f);
	EXPECT_NEAR(allocator.scale()(5), 0.25f, 1e-6f);
	const auto scale = allocator.scale();
	allocator.allocate();
	EXPECT_EQ(allocator.scale(), scale);
}
