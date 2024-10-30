This runs a shuffler simulation that I use to optimize the drive system for my robots. Everyone else is welcome to use this to optimize their robots, but please give credit if you do.

A shuffler system uses offset cams so the robot will "walk" instead of roll.

Here is a side by side of my simulation next to actual battlebots shufflers (shufflers found on midwestrobotcombat.com) 
<img width="410" alt="image" src="https://github.com/user-attachments/assets/c772c5df-d358-4c97-9111-1db1c8cc2e6f"> [](https://github-production-user-asset-6210df.s3.amazonaws.com/141258998/381576067-e3b1a8d0-ecb1-4ee1-b49f-d12c1178b6fe.png?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20241030%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20241030T135352Z&X-Amz-Expires=300&X-Amz-Signature=85c2f2f4ab8ee8488a62e09dda657641966136ba1679fecd947fed852791f848&X-Amz-SignedHeaders=host)![image](https://github.com/user-attachments/assets/88223700-fda4-4477-84cd-d11f242d5551)


The simulation allows users to adjust the following parameters:
<img width="1263" alt="image" src="https://github.com/user-attachments/assets/c4424ba4-1e38-4418-87b5-3e24e666c7cd">
The system returns the lowest point variance, and 360 degree stepover <img width="208" alt="image" src="https://github.com/user-attachments/assets/02477fe7-e13d-4443-be31-a3dcd2d3d19c">. 

Lowest point variance measures the vibration of the robot by calculating how much the robot "bounces" as it walks.
360 degree stepover is the distance that the robot travels per 360 degree rotation of the camshaft. This will determine the robot's speed and the necessary motor torque to operate.

