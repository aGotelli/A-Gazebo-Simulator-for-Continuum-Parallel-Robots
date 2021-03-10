#ifndef SOFT_BODY_SHAPE_LISTENER_H
#define SOFT_BODY_SHAPE_LISTENER_H

#include <rod_ivp_gazebo/shape_msg.h>
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

private:
  ShapeMsg msg;
  std::thread listener;
  std::chrono::milliseconds timeout;
  bool running = true;

  void listening_loop();
};

}

#endif // SOFT_BODY_SHAPE_LISTENER_H
