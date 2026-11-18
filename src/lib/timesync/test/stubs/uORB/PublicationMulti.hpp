#pragma once
#include <uORB/topics/timesync_status.h>
#define ORB_ID(name) 0
namespace uORB {
template<typename T> class PublicationMulti {
public:
 explicit PublicationMulti(int) {}
 void publish(const T &message) { test_last_status = message; }
};
}
