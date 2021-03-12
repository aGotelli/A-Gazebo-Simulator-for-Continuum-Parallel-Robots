#ifndef SOFT_BODY_SHAPE_MSG_H
#define SOFT_BODY_SHAPE_MSG_H

#include <array>
#include <map>
#include <iostream>
#include <vector>

namespace soft_body_demo
{

using Coefs = std::array<double, 4>;
using History = std::array<double, 100>;

struct ShapeMsg
{
    History x, y, z;

};
}

#endif // SOFT_BODY_SHAPE_MSG_H
