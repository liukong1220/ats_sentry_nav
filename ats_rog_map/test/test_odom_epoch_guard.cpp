// Copyright 2026

#include <gtest/gtest.h>
#include "ats_rog_map/odom_epoch_guard.hpp"

TEST(OdomEpochGuard, UnknownIdentityAndRetiredStatusFailClosed)
{
  ats_rog_map::OdomEpochGuard guard;
  EXPECT_FALSE(guard.accepts(100));
  EXPECT_FALSE(guard.update(0, 0, 100, 100, false));
  EXPECT_TRUE(guard.update(10, 1, 100, 101, false));
  EXPECT_FALSE(guard.accepts(99));
  EXPECT_TRUE(guard.accepts(100));
  EXPECT_FALSE(guard.update(9, 2, 200, 201, false));
  EXPECT_EQ(guard.instance(), 10U);
  EXPECT_EQ(guard.epoch(), 1U);
}

TEST(OdomEpochGuard, FrontendChangeRevokesInFlightWorkAndRejectsOldCloud)
{
  ats_rog_map::OdomEpochGuard guard;
  ASSERT_TRUE(guard.update(10, 1, 100, 101, false));
  const auto old_revision = guard.revision();
  EXPECT_TRUE(guard.update(20, 2, 200, 201, false));
  EXPECT_GT(guard.revision(), old_revision);
  EXPECT_FALSE(guard.accepts(199));
  EXPECT_TRUE(guard.accepts(200));
  // A map correction only changes the global localization epoch, which is
  // deliberately absent from this storage identity contract.
  EXPECT_FALSE(guard.update(20, 2, 200, 202, false));
  EXPECT_EQ(guard.epoch(), 2U);
  EXPECT_FALSE(guard.update(10, 1, 100, 150, false));
  EXPECT_EQ(guard.instance(), 20U);
}

TEST(OdomEpochGuard, ConcurrentSourceLatchRevokesAdmission)
{
  ats_rog_map::OdomEpochGuard guard;
  ASSERT_TRUE(guard.update(10, 1, 100, 101, false));
  const auto revision = guard.revision();
  EXPECT_FALSE(guard.update(10, 1, 100, 102, true));
  EXPECT_FALSE(guard.accepts(103));
  EXPECT_GT(guard.revision(), revision);
}
