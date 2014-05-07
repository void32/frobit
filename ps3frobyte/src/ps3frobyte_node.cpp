#include "ps3frobyte.h"
int main(int argc, char **argv){
        ros::init(argc, argv, "PS3frobyte");
        PS3frobyte *moveturtle = new PS3frobyte();
        ros::Rate loop_rate(10);
        while (ros::ok()){
                moveturtle->updateDeadman();
                moveturtle->updateVel();

                ros::spinOnce();
                loop_rate.sleep();
        }
  return 0;
}

