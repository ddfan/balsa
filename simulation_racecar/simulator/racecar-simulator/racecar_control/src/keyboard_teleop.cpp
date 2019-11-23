/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WVU Interactive Robotics Laboratory nor
*     the names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ncurses.h>
#include <cmath>

#define _USE_MATH_DEFINES // for math constants (e.g., M_PI)

// keys (LOW = lowercase)
#define LOW_W 119
#define LOW_D 100
#define LOW_S 115
#define LOW_A 97
#define LOW_Q 113
#define SPACEBAR 32
#define CTRL_C 3
#define ZERO 48
#define ONE 49
#define NINE 57

#define INIT_TIMEOUTS 25
#define SPEED_INCR 0.1 // speed increment (m/s)
#define ANGLE_INCR M_PI/8 // steering angle increment (radians)

#define MAX_STEER_ANGLE M_PI/2

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "keyboard_teleop_node");
    ros::NodeHandle nh;

    ros::Rate loopRate(50); // Hz

    ros::Publisher pub_ackermann_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/ackermann_cmd",1);

    ackermann_msgs::AckermannDriveStamped msg_ack_cmd;

    int ch;
    int num_timeouts = INIT_TIMEOUTS;

    float speed = 0; // m/s
    float steering_angle = 0; // radians

    bool done = false;

    initscr();			/* Start curses mode 		*/
    raw();				/* Line buffering disabled	*/
    keypad(stdscr, TRUE);		/* We get F1, F2 etc..		*/
    timeout(50);
    noecho();			/* Don't echo() while we do getch */
    resizeterm(24, 90);
    while(!done && ros::ok())
    {
        ch = getch();			/* If raw() hadn't been called
        * we have to press enter before it
        * gets to the program 		*/

        switch(ch)
        {

            //drive control
            case LOW_W:
                num_timeouts = 0;

                if (!speed) speed = SPEED_INCR;

                clear();
                printw("FORWARD!!");
                msg_ack_cmd.drive.speed = speed;
                msg_ack_cmd.drive.steering_angle = 0;
                break;

            case LOW_D:
                num_timeouts = 0;

                if (!steering_angle) steering_angle = ANGLE_INCR;

                clear();
                printw("RIGHT ROTATE");
                msg_ack_cmd.drive.steering_angle = -steering_angle;
                msg_ack_cmd.drive.speed = speed;
                break;

            case LOW_A:
                num_timeouts = 0;

                if (!steering_angle) steering_angle = ANGLE_INCR;

                clear();
                printw("LEFT ROTATE");
                msg_ack_cmd.drive.steering_angle = steering_angle;
                msg_ack_cmd.drive.speed = speed;
                break;

            case LOW_S:
                num_timeouts = 0;

                if (!speed) speed = SPEED_INCR;

                clear();
                printw("BACKWARD!!");
                msg_ack_cmd.drive.speed = -speed;
                msg_ack_cmd.drive.steering_angle = 0;
                break;

            case KEY_UP: // increase speed
                // TODO: may need to account for max speed
                speed += SPEED_INCR;
                msg_ack_cmd.drive.speed = speed;
                clear();
                printw("Speed changed to: %i", speed);
                continue;

            case KEY_DOWN: // decrease speed
                speed -= SPEED_INCR;

                if (speed < 0)
                    speed = 0;

                msg_ack_cmd.drive.speed = speed;
                clear();
                printw("Speed changed to: %i", speed);
                continue;

            case KEY_RIGHT: // increase steering angle
                steering_angle += ANGLE_INCR;

                if (steering_angle > MAX_STEER_ANGLE)
                    steering_angle = MAX_STEER_ANGLE;

                msg_ack_cmd.drive.steering_angle = steering_angle;
                clear();
                printw("Steering angle changed to: %f", steering_angle);
                continue;

            case KEY_LEFT: // decrease steering angle
                steering_angle -= ANGLE_INCR;

                if (steering_angle < 0)
                    steering_angle = 0;

                msg_ack_cmd.drive.steering_angle = steering_angle;
                clear();
                printw("Steering angle changed to: %f", steering_angle);
                continue;


            case SPACEBAR: // stop/reset
                num_timeouts = 0;
                clear();
                printw("Reset speed & steering angle to 0");
                speed = 0;
                steering_angle = 0;

                msg_ack_cmd.drive.speed = 0;
                msg_ack_cmd.drive.steering_angle = 0;
                break;


            case LOW_Q: //if Q, close everything ROS
                msg_ack_cmd.drive.speed = 0;
                msg_ack_cmd.drive.steering_angle = 0;
                pub_ackermann_cmd.publish(msg_ack_cmd);
                system("rosnode kill -a && sleep 5 && kill -2 $( ps -C roslaunch -o pid= ) && sleep 2 && kill -2 $( ps -C roscore -o pid= )");
                return 0;

            case CTRL_C: // if control-c, close keyboard node
                printw("Shutting Down");
                msg_ack_cmd.drive.speed = 0;
                msg_ack_cmd.drive.steering_angle = 0;
                pub_ackermann_cmd.publish(msg_ack_cmd);
                done = true;
                break;

            //timeout
            case -1: //timeout
                ++num_timeouts;
                if(num_timeouts > 10)
                {
                    clear();
                }
                if(num_timeouts > INIT_TIMEOUTS)
                {
                    printw("Current drive speed (m/s): %f\n", speed);
                    printw("Current steering angle (radians): %f\n\n", steering_angle);

                    printw("Key 'up' increases speed, Key 'down' decreases speed\n");
                    printw("Key 'right' increases steering angle, Key 'left' decreases steering angle\n");
                    printw("Key 'spacebar' resets speed & steering angle to 0\n\n");

                    printw("Key 'w' drives forward, Key 's' drives backward\n");
                    printw("Key 'd' turns right, Key 'a' turns left\n");
                    printw("Key 'q' will close all rosnodes and end keyboard teleop node\n");
                    printw("'Control + c' will close just keyboard control\n");
                }

                msg_ack_cmd.drive.speed = 0;
                msg_ack_cmd.drive.steering_angle = 0;
                break;


            default: // unknown key press
                num_timeouts = 0;
                clear();
                printw("The pressed key is unknown %i", ch);
                msg_ack_cmd.drive.speed = 0;
                msg_ack_cmd.drive.steering_angle = 0;
                break;
        }

        // quick drive speed control using number line on keyboard
        if(ch >= ZERO && ch <= NINE)
        {
            speed =  ((ch - ONE)+1)/10.0;
            clear();
            printw("Speed Change %i", speed);
        }

        pub_ackermann_cmd.publish(msg_ack_cmd);

        ros::spinOnce();
        loopRate.sleep();

    }


    refresh();			/* Print it on to the real screen */
    getch();			/* Wait for user input */
    endwin();			/* End curses mode		  */

    return 0;
}
