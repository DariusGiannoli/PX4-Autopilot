# SPDX-License-Identifier: BSD-3-Clause
px4_add_module(
	MODULE modules__morphing_arm_publisher
	MAIN morphing_arm_publisher
	SRCS MorphingArmPublisher.cpp MorphingArmPublisher.hpp ArmCommandState.hpp
	MODULE_CONFIG module.yaml
	DEPENDS px4_work_queue
)
px4_add_functional_gtest(SRC MorphingArmPublisherTest.cpp
	COMPILE_FLAGS -DMODULE_NAME=\"morphing_arm_publisher_test\"
	LINKLIBS modules__morphing_arm_publisher ActuatorEffectiveness ControlAllocation)
