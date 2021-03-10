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
    // a shape is described by a set of 3 3rd-order polynomials
    // they describe the curve in X, Y and Z
    Coefs xc, yc, zc;
    History x, y, z;


    void print(std::string prefix) const
    {
      std::map<std::string, Coefs> polynomials
      {
        {"X", xc},{"Y", yc},{"Z", zc}
      };
      // just display for now
      for(const auto &[key, coefs]: polynomials)
      {
        std::cout << prefix << " " << key << ": ";
        for(const auto &coef: coefs)
          std::cout << coef << " ";
        std::cout << std::endl;
      }
    }    

    void print2() const
    {
      std::cout << "X history : " << std::endl;
      for(const auto &xi: x)
        std::cout << xi << " ";
      std::cout << std::endl;
      std::cout << "Y history : " << std::endl;
      for(const auto &xi: x)
        std::cout << xi << " ";
      std::cout << std::endl;
      std::cout << "Z history : " << std::endl;
      for(const auto &xi: x)
        std::cout << xi << " ";
      std::cout << std::endl;
    }
};
}

#endif // SOFT_BODY_SHAPE_MSG_H
