// SPDX-License-Identifier: BSD-3-Clause
// Real uORB subscriptions with a deterministic low-uptime clock. Desktop
// CLOCK_MONOTONIC normally starts above these early-boot boundary values.
#include <gtest/gtest.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

namespace
{
bool fake_clock_enabled = false;
hrt_abstime fake_now = 0;
}

extern "C" hrt_abstime __real_hrt_absolute_time();
extern "C" hrt_abstime __wrap_hrt_absolute_time()
{
	return fake_clock_enabled ? fake_now : __real_hrt_absolute_time();
}

class SubscriptionIntervalTest : public ::testing::TestWithParam<hrt_abstime>
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		fake_now = 0;
		fake_clock_enabled = true;
	}

	void TearDown() override { fake_clock_enabled = false; }

	void publish(uint32_t instance)
	{
		parameter_update_s message{};
		message.timestamp = fake_now;
		message.instance = instance;
		ASSERT_TRUE(_publication.publish(message));
	}

	uORB::Publication<parameter_update_s> _publication{ORB_ID(parameter_update)};
};

TEST_P(SubscriptionIntervalTest, FirstCopyPreservesFutureNotificationAndRateLimit)
{
	constexpr uint32_t interval = 1'000'000;
	fake_now = GetParam();
	publish(1);
	uORB::SubscriptionInterval subscription{ORB_ID(parameter_update), interval};
	parameter_update_s update{};
	// EKF2 forces a first copy before registering its IMU callback.
	ASSERT_TRUE(subscription.copy(&update));
	ASSERT_EQ(update.instance, 1u);

	fake_now = GetParam() + interval - 1;
	publish(2);
	EXPECT_FALSE(subscription.updated());
	fake_now += 1;
	EXPECT_TRUE(subscription.updated());
	ASSERT_TRUE(subscription.update(&update));
	EXPECT_EQ(update.instance, 2u);
	EXPECT_FALSE(subscription.updated());

	fake_now += interval - 1;
	publish(3);
	EXPECT_FALSE(subscription.updated());
	fake_now += 1;
	ASSERT_TRUE(subscription.update(&update));
	EXPECT_EQ(update.instance, 3u);
}

INSTANTIATE_TEST_SUITE_P(BootUptime, SubscriptionIntervalTest,
	::testing::Values(hrt_abstime{0}, hrt_abstime{1}, hrt_abstime{100'000},
			 hrt_abstime{999'999}, hrt_abstime{1'000'000}));

TEST_F(SubscriptionIntervalTest, EarlySubscribersKeepIndependentIntervals)
{
	constexpr uint32_t interval = 1'000'000;
	parameter_update_s update{};
	fake_now = 100'000;
	publish(1);
	uORB::SubscriptionInterval first{ORB_ID(parameter_update), interval};
	ASSERT_TRUE(first.copy(&update));
	fake_now = 500'000;
	uORB::SubscriptionInterval second{ORB_ID(parameter_update), interval};
	ASSERT_TRUE(second.copy(&update));
	fake_now = 1'100'000;
	publish(2);
	ASSERT_TRUE(first.update(&update));
	EXPECT_EQ(update.instance, 2u);
	EXPECT_FALSE(second.updated());
	fake_now = 1'500'000;
	ASSERT_TRUE(second.update(&update));
	EXPECT_EQ(update.instance, 2u);
}

TEST_F(SubscriptionIntervalTest, LateFirstCopyRetainsCatchupBehavior)
{
	constexpr uint32_t interval = 1'000'000;
	fake_now = 2'000'000;
	publish(1);
	uORB::SubscriptionInterval subscription{ORB_ID(parameter_update), interval};
	parameter_update_s update{};
	ASSERT_TRUE(subscription.copy(&update));
	publish(2);
	ASSERT_TRUE(subscription.update(&update));
	EXPECT_EQ(update.instance, 2u);
	publish(3);
	EXPECT_FALSE(subscription.updated());
	fake_now += interval;
	ASSERT_TRUE(subscription.update(&update));
	EXPECT_EQ(update.instance, 3u);
}
