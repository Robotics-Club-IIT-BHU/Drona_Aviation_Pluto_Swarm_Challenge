<h1 align="center"> Drona-Aviation-Inter-IIT-2023 </h1>

<p align="center">
  <img src="https://astirtech.com/wp-content/uploads/2021/09/product-page-banner-new.png" alt="Loading image..."/>
</p>

## Abstract
Pluto is a lightweight programmable drone created by <a href="https://www.dronaaviation.com/">Drona Aviation</a> to bring a low-end solution for enthusiasts interested in working on the drone. Supporting their ideas, we have made a Python Wrapper to control the drone using either Linux/Windows. Further, we developed a localization pipeline by detecting an ArUco marker placed on top of it. Pluto communicates using MSP Packets and takes input in various data packet forms. Moreover, the drone has an inbuilt PID controlled, on top of which we have implemented two PID controllers for Position and Height control. We added a trajectory traversal system which helps the drone to traverse on a polygonal trajectory of defined shape. This project was done as a solution to the [PS](https://drive.google.com/file/d/1hHxGhYIEn29oFeoYI5EiU3N7TgcLyP01/view?usp=sharing) given by Drona Aviation in a High Prep event in Inter IIT Tech Meet 11.0 at IIT Kanpur.

### For demos, refer this [link](https://drive.google.com/drive/folders/1qRpV-A7ePrWSwaF6nJWXYUjQ2Q_1TDXh?usp=sharing)

---
## Installation:

1. <b>Clone this repository:</b>
```
git clone https://github.com/Robotics-Club-IIT-BHU/Drona-Aviation-Inter-IIT-2023.git
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

## For Keyboard Teleop using Python Wrapper:

1. Move inside teleop folder inside the comm folder
```
cd comm/teleop
```
2. Run key_command file to start controller
```
python key_command.py
```
## For Rectangular Motion:
1. Run the main.py file
```
python main.py
```
## For Swarm Motion:
1. Move inside Task-3 directory
```
cd Task-3
```
2. Install Dependencies
```
pip install .
```
3. Run follow.py
```
python follow.py
```
