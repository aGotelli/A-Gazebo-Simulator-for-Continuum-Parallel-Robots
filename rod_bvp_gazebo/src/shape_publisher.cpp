#include <rod_bvp_gazebo/shape_publisher.h>
#include <iostream>
#include <map>
namespace soft_body_demo
{

ShapePublisher::ShapePublisher(std::chrono::milliseconds timeout)
  : sock(ctx, zmq::socket_type::pair), timeout(timeout)
{
  sock.setsockopt( ZMQ_LINGER, 0 );
  sock.bind("ipc://@rod_ivp_visual");
}

ShapePublisher::~ShapePublisher()
{
  sock.close();
  ctx.close();
}

void ShapePublisher::publish(const ShapeMsg &msg)
{
  static zmq::pollitem_t poll_out{nullptr, 0, ZMQ_POLLOUT, 0};
  poll_out.socket = static_cast<void*>(sock);
  zmq::poll(&poll_out, 1, timeout);

  if(poll_out.revents & ZMQ_POLLOUT)
  {
    zmq::message_t zmsg(&msg, sizeof(msg));
    sock.send(zmsg, zmq::send_flags::none);
  }
}
}
