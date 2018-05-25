# ECE 470 Final Project: * https://www.youtube.com/watch?v=NMZ6Y7S99F0
========================================================================================
## Week1
Steps to reproduce the results in the following videos on Mac OS macOS High Sierra for the first week: 
* https://www.youtube.com/watch?v=Q86Gz2qPCgs&feature=youtu.be 
* https://www.youtube.com/watch?v=AINoNR9a-EM&feature=youtu.be

1. Download V-rep using the following link, and unzip it
    * http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_4_0_Mac.zip

1. To start the simulator, navigate to the unzipped directory in the terminal, type:
 ```./vrep.app/Contents/MacOS/vrep```

1. Create another folder outside the Vrep folder, and move the following files into the new folder:
	```
	vrep/programming/remoteApiBindings/python/python/vrep.py
	vrep/programming/remoteApiBindings/python/python/vrepConst.py
	vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
	```
1. Add the test.py from this submission zip file to the folder from the last step

1. Add the UR3 robot arm by dragging the UR3 icon from V-rep  model browser into the scene located on the right panel

1. Move the robot:
	1. To move all joints of the robot, simply execute the python script in the terminal by typing:
		```python test.py```
	1. To move the cup on the table, modify the fifth line of the code from  
		```move_all_joint = True``` 
		to 
		```move_all_joint = False```

========================================================================================
## Week2
Steps to reproduce the results in the following video on Mac OS macOS High Sierra for the second week: 
* https://youtu.be/mMlMBneSkos

1. Open V-rep by typing the following line in the terminal under the VREP folder, and make sure the VREP is at version 3.5
	```./vrep.app/Contents/MacOS/vrep```

1. Add the week2.py and helper.py from the zip file to the vrep_code folder created last week

1. In the vrep simulator, add the UR3 robot arm, move it to position (0,0,0), and attach a dummy frame to the end-effector position. Add another dummy frame, place it at any random location

1. Execute the week2.py script by typing:
 ```python3 week2.py```

 ========================================================================================
## Week3
Steps to reproduce the results in the following video on Mac OS macOS High Sierra for the third week: 
* https://www.youtube.com/watch?v=gON0NJzTZXU&feature=youtu.be

1. Open V-rep by typing the following line in the terminal under the VREP folder, and make sure the VREP is at version 3.5
	```./vrep.app/Contents/MacOS/vrep```

1. Add the week2.py and helper.py from the zip file to the vrep_code folder created last week

1. In the vrep simulator, add the UR3 robot arm, move it to position (0,0,0), and attach a dummy frame to the end-effector position. Add another dummy frame, place it at any random location

1. Execute the week3.py script by typing:
 ```python3 week3.py```
 
  ========================================================================================
## Week4
Steps to reproduce the results in the following video on Mac OS macOS High Sierra for the fourth week: 
* https://youtu.be/sl9AbGa7Hfg

1. Open V-rep by typing the following line in the terminal under the VREP folder, and make sure the VREP is at version 3.5
	```./vrep.app/Contents/MacOS/vrep```

1. Add the week4.py and helper.py from the zip file to the vrep_code folder created last week

1. To recreate the starting scene, add the week4.ttt file to your vrep_code folder

1. Open VREP, click on file on the top menu bar, select open scene, and select the .ttt file added in the previous step

1. Execute the week4.py script in a terminal by typing:
 ```python3 week4.py```
 
 
   ========================================================================================
## Week5
Steps to reproduce the results in the following video on Mac OS macOS High Sierra for the fifth week: 

1. Open V-rep by typing the following line in the terminal under the VREP folder, and make sure the VREP is at version 3.5
	```./vrep.app/Contents/MacOS/vrep```

1. Add the week5.py and helper.py from the zip file to the vrep_code folder created last week

1. To recreate the starting scene, add the week5.ttt file to your vrep_code folder

1. Open VREP, click on file on the top menu bar, select open scene, and select the .ttt file added in the previous step

1. Execute the week5.py script in a terminal by typing:
 ```python3 week5.py```


 ========================================================================================
## Final Demo
Steps to reproduce the results in the following video on Mac OS macOS High Sierra for the final demo: 

1. Open V-rep by typing the following line in the terminal under the VREP folder, and make sure the VREP is at version 3.5
	```./vrep.app/Contents/MacOS/vrep```

1. Add the final.py, helper.py, and lib.py from the zip file to the vrep_code folder 

1. To recreate the starting scene, add the final.ttt file to your vrep_code folder

1. Open VREP, click on file on the top menu bar, select open scene, and select the .ttt file added in the previous step

1. Execute the week5.py script in a terminal by typing:
 ```python3 final.py```
