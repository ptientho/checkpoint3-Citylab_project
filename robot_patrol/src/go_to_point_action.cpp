

class GoToPoint {
  // create action server "/go_to_point

  // subscribe to odometry topic

  // publish to cmd_vel
  // given odometry readings at every loop, send velocity command(control loop)

  // action message here
  // geometry_msgs/Point32 goal_pos
  //---
  // bool status
  //---
  // geometry_msgs/Point32 current_pos

  // The action server will receive a Point32 message as a goal. This Point32
  // will define the goal position [x, y, theta] for the robot.

  // As feedback, the action server publishes the current position of the robot
  // [x,y,theta] every second.

  // When the action finishes, the result will return the following:

  // A boolean with True inndicating the action finished correctly
};