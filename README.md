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
***rosrun hybrid_control_api hanoiTower.py***  