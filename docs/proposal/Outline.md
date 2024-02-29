### Enhancing Robot Learning from Demonstration through Mixed Reality Interventions



#### Equipment:

* Microsoft Hololens 2
* Franka Emika Panda

* Several cubes (3D printed) or LEGO blocks

* Two bins



#### Sorting Task

* Learning:

  * The user is not required to be co-located with the robot.

  * Several cubes or LEGO blocks are placed in front of the user and the robot.

  * The user first creates two virtual regions in Hololens 2 (e.g. one red, one blue).

  * Then the user create demonstrations by using their hands to place cubes into different regions.

  * The sorting rule is designed by the user.

  * By using the mixed reality capture, the sorting results are processed and then learned by the robot.

  * Object detection (e.g. Yolo) will be needed to judge each cube's position.

  * Generate training dataset from object detection results and learn the sorting rules (e.g. Decision Tree).

* Outcome:
  * The user can preview the robot's learning outcomes.
  * The user can ask the robot to sort all the cubes on the desk.
  * The user can ask the robot to sort the selected cube (gaze, gesture).
  * The user can query the robot learning status.
  * The user can correct the robot if it make mistakes.



#### Research Questions

* RQ1: Is it possible to leverage mixed reality capture to assist in the creation of visual demonstrations?
* RQ2: Can interventions using MR user interfaces enhance the teaching efficiency for users lacking expertise in robotics?



#### Phase 0: Implement `robomimic`

* Collect training data for the task
  * Kinesthetic teaching
  * Use XR

* Model training
* Test performance in simulation
* Test performance on real robot



#### Phase 1: Implement object detection and rule learning

* Collect training data for object detection.
* Test the performance of object detection.
* Implement conversion of object detection results .
* Use machine learning to learn the rules. 



#### Phase 2: Merge outcomes of phase 1 and phase 2

* Develop a program that bridges Hololens 2 and Franka
* Complete the basic workflow



#### Study 1: Test system performance



#### Phase 3: Implement user interface interventions

* Demonstration Editor
* Robot's moving range
* Display demonstration level
  * uncertainty
  * learned rules in natural languages

* Error correction
* Test other machine learning techniques for learning rules



#### Study 2: Compare teaching efficiency of different levels of user interventions

