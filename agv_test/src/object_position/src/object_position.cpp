#include <ros/ros.h>
#include <tf/transform_listener.h>//接收者
#include <geometry_msgs/Twist.h>//消息类型

 
int main(int argc, char** argv){
  ros::init(argc, argv, "object_position");
 
  ros::NodeHandle node;
 
 // ros::service::waitForService("spawn");//等待，直到服务“spawn”出现，只有这样才能请求此服务再产生一个turtle  此服务没有出现就一只等待
 //ros::ServiceClient add_turtle =
  //  node.serviceClient<turtlesim::Spawn>("spawn");//客户端来调用服务
 // turtlesim::Spawn srv;//这个服务对象包含请求和响应两部分。
 // add_turtle.call(srv);//调用此服务就是为了产生一个新的乌龟
 
 // ros::Publisher turtle_vel =
   // node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);//这里来决定发布的速度信息是给谁的
 
  tf::TransformListener listener;//tf订阅者  一旦这个对象创建了，就开始接收坐标系间的变化信息
 
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;//储存变换的信息
    try{

      listener.waitForTransform("/kinect2_depth_frame_ir_optical_frame", "/object_15", ros::Time(0), ros::Duration(3.0));

      listener.lookupTransform("/kinect2_depth_frame_ir_optical_frame", "/object_15",                /*//targetframe    sourceframe   time  transform  四个参数   transform是turtle1到turtle2的变换*/
                               ros::Time(0), transform);         //Time(0)就是把最新的有效信息发布出去     在broadcaseter中是将当前信息发布出去     获得两个坐标系之间的变换关系R，t
    }
    catch (tf::TransformException &ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();//休眠一下，休眠时间为1s
      continue;
    }

  ros::Publisher object_position_pub = node.advertise<geometry_msgs::Point>("/object_position",10);

 
      geometry_msgs::Point position;

      position.x = transform.getOrigin().x();

      position.y = transform.getOrigin().y();

      position.z = transform.getOrigin().z();

      object_position_pub.publish(position);
    //geometry_msgs::Twist vel_msg;   //从transform中得到消息，然后发布给turtle2
     //vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),//实例化这个消息
      //                          transform.getOrigin().x());
    // vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
     //                                  pow(transform.getOrigin().y(), 2));//pow()是平方函数
   //turtle_vel.publish(vel_msg);//发布消息使turtle2运动
 
    rate.sleep();
  }
  return 0;
};
 
/*调用服务spawn，然后再生一只乌龟*/
/*zhe li zui hou yong Rviz lai guankan*/
