#include <Example.hpp>
#include <ros/ros.h>

namespace Example
{
IKEMURA::IKEMURA(const std::string &word) : word(word)
{
}

void IKEMURA::sayHi()
{
   ROS_INFO_STREAM("Hi");
}

void IKEMURA::sayBye()
{
   ROS_INFO("Bye");
}

void IKEMURA::sayWord()
{
   ROS_INFO("%s", word.c_str());
}

}  // namespace Example
