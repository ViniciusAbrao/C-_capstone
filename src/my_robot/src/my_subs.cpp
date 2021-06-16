#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <memory>

class MyMovableClass
{
private:
  
    int _size;

public:
  
    float *_data;
  
    MyMovableClass(size_t size) // constructor
    {
        _size = size;
        _data = new float[_size];
        std::cout << "CREATING instance of MyMovableClass at " << this << " allocated with size = " << _size*sizeof(float)  << " bytes" << std::endl;
    }

    ~MyMovableClass() // 1 : destructor
    {
        std::cout << "DELETING instance of MyMovableClass at " << this << std::endl;
        delete[] _data;
    }
    
    MyMovableClass(const MyMovableClass &source) // 2 : copy constructor
    {
        _size = source._size;
        _data = new float[_size];
        *_data = *source._data;
        std::cout << "COPYING content of instance " << &source << " to instance " << this << std::endl;
    }
    
    MyMovableClass &operator=(const MyMovableClass &source) // 3 : copy assignment operator
    {
        std::cout << "ASSIGNING content of instance " << &source << " to instance " << this << std::endl;
        if (this == &source)
            return *this;
        delete[] _data;
        _data = new float[source._size];
        *_data = *source._data;
        _size = source._size;
        return *this;
    }
    
    MyMovableClass(MyMovableClass &&source) // 4 : move constructor
    {
        std::cout << "MOVING (c’tor) instance " << &source << " to instance " << this << std::endl;
        _data = source._data;
        _size = source._size;
        source._data = nullptr;
        source._size = 0;
    }
    
    MyMovableClass &operator=(MyMovableClass &&source) // 5 : move assignment operator
    {
        std::cout << "MOVING (assign) instance " << &source << " to instance " << this << std::endl;
        if (this == &source)
            return *this;

        delete[] _data;

        _data = source._data;
        _size = source._size;

        source._data = nullptr;
        source._size = 0;

        return *this;
    }
   
  /**
 * This function receipts laser messages over the ROS system.
 */
  void counterCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //ROS_INFO("LaserScan (val,angle)=(%f,%f", msg->range_min,msg->angle_min);
    //ROS_INFO("LaserScan in front=(%f", msg->ranges[360]);
    *_data=msg->ranges[360];
  }
  
};

class PrintData
{
  private:
    float progress;
    int barWidth;
    int pos;
    int i;
  
  public:
    void print_data(std::shared_ptr<MyMovableClass> Ptr){
      //ROS_INFO("LaserScan in front=(%f", *_data);
      //std::cout << "LaserScan in front [meters]:" << *_data << std::endl;
      progress=1-(*Ptr->_data/30); //normalized distance by the maximum range of laser (30 meters)
      barWidth = 50;
      std::cout << "30m[";
      pos = barWidth * progress;
      for (i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
      }
      std::cout << "]0m- DistToObst: " << int(*Ptr->_data * 100) << "cm \r";
      std::cout.flush();
    }
  
};

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  std::shared_ptr<MyMovableClass> sharedPtr = std::make_shared<MyMovableClass>(1);
  
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, boost::bind(&MyMovableClass::counterCallback, sharedPtr, _1));

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  
  PrintData ProgressBar;
  
  ros::Rate loop_rate(10);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    ProgressBar.print_data(sharedPtr);
  }

  std::cout << std::endl;
  return 0;
}