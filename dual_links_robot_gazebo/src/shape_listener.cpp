#include <dual_links_robot_gazebo/shape_listener.h>


namespace soft_body_demo
{

ShapeListener::ShapeListener(std::chrono::milliseconds timeout) : timeout(timeout),
  sock(ctx, zmq::socket_type::pair)
{
  listener = std::thread([&](){listening_loop();});
  for(auto& x : msg.x)
    x = 0.0;
  for(auto& y : msg.y)
    y = 0.0;
  for(auto& z : msg.z)
    z = 0.0;
}

ShapeListener::~ShapeListener()
{
  running = false;
  listener.join();
}

void ShapeListener::listening_loop()
{
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
