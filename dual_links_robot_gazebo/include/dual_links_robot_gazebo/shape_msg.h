#ifndef DUAL_LINK_SHAPE_MSG_H
#define DUAL_LINK_SHAPE_MSG_H

#include <array>
#include <map>
#include <iostream>
#include <vector>

namespace soft_body_demo
{

//using Coefs = std::array<double, 4>;
using History = std::array<double, 100>;

//struct Pose {
//  double x, y, z;
//};

//struct UniqueShape {
//  History x, y, z;
//};

struct ShapeMsg
{
   History x, y, z;
};
}

#endif // DUAL_LINK_SHAPE_MSG_H
