# Hybrid Control

A hybrid control between a Jacobian Neural Network and Thormang3's Jacobian to calculate Inverse Kinematics

## How to Run

***cd catkin_ws/src***

***git clone https://github.com/ricardoGrando/hybrid_control_api.git***   

***cd catkin_ws && catkin_make -j4***   

***roscore*** 

Open new Terminal

***source ~/catkin_ws/devel.setup.bash***  

***rosrun hybrid_control_api fk_and_jacobian_server***  

Open new Terminal

***chmod +x ~/catkin_ws/src/hybrid_control_api/scripts/hanoiTower.py***  

***rosrun hybrid_control_api hanoiTower.py***  

[![Watch the video](https://github.com/ricardoGrando/hybrid_control_api/blob/master/git_data/video.png)](https://www.youtube.com/watch?v=O2vynMscfGA&t=826s)

