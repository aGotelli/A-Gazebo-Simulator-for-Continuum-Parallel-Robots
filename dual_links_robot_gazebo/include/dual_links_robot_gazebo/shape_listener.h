#ifndef SOFT_BODY_SHAPE_LISTENER_H
#define SOFT_BODY_SHAPE_LISTENER_H
#include <dual_links_robot_gazebo/zmq.hpp>

#include <dual_links_robot_gazebo/shape_msg.h>
#include <chrono>
#include <thread>

namespace soft_body_demo
{

class ShapeListener
{
public:
  ShapeListener(std::chrono::milliseconds timeout = std::chrono::milliseconds{10});
  ~ShapeListener();

  inline ShapeMsg lastVisual() const
  {
    return msg;
  }

  void defineSock(const std::string& sock_name)
  {
    sock.setsockopt( ZMQ_LINGER, 0 );
    sock.connect(sock_name);
  }

private:
  ShapeMsg msg;
  std::thread listener;
  std::chrono::milliseconds timeout;
  bool running = true;
  zmq::context_t ctx;
  zmq::socket_t sock;

  void listening_loop();

};

}

#endif // SOFT_BODY_SHAPE_LISTENER_H
