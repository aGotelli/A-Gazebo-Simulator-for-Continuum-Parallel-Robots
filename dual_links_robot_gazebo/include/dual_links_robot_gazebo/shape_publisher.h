#ifndef SOFT_BODY_SHAPE_PUBLISHER_H
#define SOFT_BODY_SHAPE_PUBLISHER_H

#include <dual_links_robot_gazebo/zmq.hpp>
#include <dual_links_robot_gazebo/shape_msg.h>

namespace soft_body_demo
{

class ShapePublisher
{
public:
  ShapePublisher(std::chrono::milliseconds timeout = std::chrono::milliseconds{10});
  ~ShapePublisher();

  void defineSock(const std::string& sock_name);

  void publish(const ShapeMsg &msg);

private:
  zmq::context_t ctx;
  zmq::socket_t sock;
  std::chrono::milliseconds timeout;
};
}

#endif // SOFT_BODY_SHAPE_PUBLISHER_H
