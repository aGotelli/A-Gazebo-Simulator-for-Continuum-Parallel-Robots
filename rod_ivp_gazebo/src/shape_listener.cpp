#include <rod_ivp_gazebo/shape_listener.h>
#include <rod_ivp_gazebo/zmq.hpp>

namespace soft_body_demo
{

ShapeListener::ShapeListener(std::chrono::milliseconds timeout) : timeout(timeout)
{
  listener = std::thread([&](){listening_loop();});
  msg.xc = msg.yc = {0,0,0,0};
  msg.zc = {0,-1,0,0};
}

ShapeListener::~ShapeListener()
{
  running = false;
  listener.join();
}

void ShapeListener::listening_loop()
{
  zmq::context_t ctx;
  zmq::socket_t sock(ctx, zmq::socket_type::pair);
  sock.setsockopt( ZMQ_LINGER, 0 );

  sock.connect("ipc://@rod_ivp_visual");

  zmq::pollitem_t poll_in{nullptr, 0, ZMQ_POLLIN, 0};
  poll_in.socket = static_cast<void*>(sock);

  while(running)
  {
    zmq::poll(&poll_in, 1, timeout);

    if(poll_in.revents & ZMQ_POLLIN)
    {
      zmq::message_t zmsg;
      (void)sock.recv(zmsg);
      msg = *(static_cast<ShapeMsg*>(zmsg.data()));
    }
  }
  sock.close();
  ctx.close();
}
}
