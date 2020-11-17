#include <Example.hpp>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace Example;

constexpr int MAX_NOISE = 30;
int main(int ac, char **av)
{
   ros::init(ac, av, "example_node");
   ros::NodeHandle node;

   ros::Time::init();
   srand(ros::Time::now().sec);

   IKEMURA ikemura("This is an example");
   ROS_INFO_STREAM("OpenCV version: " << CV_VERSION);

   ikemura.sayHi();

   cv::Mat img = cv::Mat::ones(cv::Size(1200, 900), CV_8UC3);

   int cnt = 0;
   while (ros::ok() && cnt <= 2000)
   {
      ikemura.sayWord();

      for (int i = 0; i < img.rows; i++)
      {
         cv::Vec3b *ptr = img.ptr<cv::Vec3b>(i);

         for (int j = 0; j < img.cols; j++)
         {
            uint8_t b = ptr[j][0];
            uint8_t g = ptr[j][1];
            uint8_t r = ptr[j][2];

            unsigned randomVar = rand() % 10;
            if (randomVar == 5)
            {
               ptr[j][0] = (b <= 255 - MAX_NOISE) ? b + rand() % MAX_NOISE : b;
               ptr[j][1] = (g <= 255 - MAX_NOISE) ? g + rand() % MAX_NOISE : g;
               ptr[j][2] = (r <= 255 - MAX_NOISE) ? r + rand() % MAX_NOISE : r;
            }
            else if (randomVar == 7)
            {
               ptr[j][2] -= (r >= MAX_NOISE ? rand() % MAX_NOISE : 0);
               ptr[j][1] -= (g >= MAX_NOISE ? rand() % MAX_NOISE : 0);
               ptr[j][0] -= (b >= MAX_NOISE ? rand() % MAX_NOISE : 0);
            }
         }
      }

      cv::imshow("image window", img);
      if (cv::waitKey(2) == 'q')
         break;

      if (rand() % 20 == 5)
         img += 1;

      cnt++;
   }

   cv::destroyAllWindows();

   ikemura.sayBye();
}
