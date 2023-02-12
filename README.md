<h1 align="center"> Drona-Aviation-Inter-IIT-2023 </h1>

<p align="center">
  <img src="https://astirtech.com/wp-content/uploads/2021/09/product-page-banner-new.png" alt="Loading image..."/>
</p>
<h5 align="center">
Pluto is a lightweight programmable drone created by <a href="https://www.dronaaviation.com/">Drona Aviation</a> to bring a low-end solution for enthusiasts interested in working on the drone. Supporting their ideas, we have made this python wrapper to control and localize the pluto drone using computer vision and a lidar placed at the bottom of the drone. Pluto drone communicates using msp packets and takes input in various data packet forms. Moreover, the drone has an inbuilt pid stabilizer, on top of which we have implemented two pid controllers to maintain the height and position of it in the camera frame.
</h5>


---
## Setting up the project:

1. <b>Clone this repository:</b>
```
git clone https://github.com/Robotics-Club-IIT-BHU/Drona-Aviation-Inter-IIT-2023
```
2. <b>Initial setup:</b> 
  - Install [virtual environment](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/)  
  - Create a new environment `python3 -m venv venv` 
  - Activate virtual environment 
    - linux: `source venv/bin/activate`
    - windows: `./venv/Scripts/activate`
  - Move inside the main folder `cd Drona-Aviation-Inter-IIT-2023` 
3. <b>Install dependencies: </b>
```
pip install requirements.txt
``` 
4. <b>You are ready to go:</b> (Connect your pc with drone's wifi)
```
python main.py
```

---
## For keyboard teleop using python wrapper:

1. Move inside teleop folder inside the comm folder
```
cd comm/teleop
```
2. Run key_command file to start controller
```
python key_command.py
```
