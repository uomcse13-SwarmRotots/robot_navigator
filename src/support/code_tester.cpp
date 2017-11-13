#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

boost::thread* thread;

void test(){
    while(true){
        std::cout << "hello\n" ;
    }
}

int main(int argc, char** argv)
{
    thread = new boost::thread(boost::bind(test));
    while(true){
        std::string s;
        std::cin >> s;
        if(s=="e"){
            thread->interrupt() ;
            break;
        }
    }
    //my_thread.join() ;
    return 0;
}


