# Control-for-Obstacle-Avoidance-by-Nonholonomic-Mobile-Robots
This repository containts Matlab codes associated with the following research paper:

@article{lafmejani2020adaptation,
  title={Adaptation of Gradient-Based Navigation Control for Holonomic Robots to Nonholonomic Robots},
  author={Lafmejani, Amir Salimi and Farivarnejad, Hamed and Berman, Spring},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={1},
  pages={191--198},
  year={2020},
  publisher={IEEE}
}


Artificial potential fields are commonly used to synthesize controllers that navigate Wheeled Mobile Robots (WMRs) through known environments with obstacles. A special type of potential field called a navigation function can be used to design controllers that guarantee collision-free robot navigation to a target position. These functions have been developed specifically for point-mass models of holonomic robots. In this project, we propose a navigation function-based controller for collision-free position control of a nonholonomic WMR in an environment with known convex obstacles and boundary. This smooth, continuous, nonlinear controller is able to drive a nonholonomic robot to a target position while preventing collisions of the robot with the obstacles and boundary. Unlike existing control approaches for nonholonomic WMR navigation, our proposed controller (1) facilitates the implementation of any navigation function-based technique for obstacle avoidance on a nonholonomic WMR; (2) uses trigonometric functions of the robot’s heading angle in quaternion form rather than direct measurements of this angle, which can result in undesired large changes in the control input; and (3) produces smooth transient performance characteristics and fast convergence of the WMR to the target position. We demonstrate the effectiveness of our controller for various initial robot configurations and environments through MATLAB simulations and physical experiments with a commercial nonholonomic WMR, the Turtlebot3 Burger robot.
