# LfD on Franka



## Control

### Status lights

- Red: reach joint limits
- Yellow: singular configuration



### Torque Direction

- counter clockwise `+`

- clockwise `-`





## Tasks

* Detect object's location and pose
  * Depth Camera
  * Tag
* Collect trajectory data
* Save as hdf5
* Training

* Run the policy





## Tasks

Learning from Demonstration (LfD) can be particularly useful for robotic arm tasks that are complex, involve human-like dexterity, or are challenging to program explicitly using traditional methods. Here are some tasks where LfD has shown promise or could be beneficial for robotic arms:

1. **Complex Manipulation**: Tasks that involve manipulating objects in ways that are hard to define algorithmically. Examples include:
   - Tying knots
   - Folding clothes
   - Assembling intricate parts
2. **Human-Robot Collaboration**: Tasks where the robot needs to work alongside humans and potentially adapt to human movements or cues. Examples include:
   - Handing over tools in a cooperative manner
   - Collaboratively assembling a product with a human partner
3. **Tasks in Unstructured Environments**: In environments where the context can change frequently, or it's hard to have a predefined structure, LfD can be beneficial. Examples include:
   - Picking fruits from trees where each fruit's position and orientation might vary
   - Sorting recyclables where items can be of various shapes, sizes, and materials
4. **Artistic or Craft Tasks**: Tasks that have a level of artistic touch or require dexterity that's typically associated with human craftsmanship. Examples include:
   - Painting or drawing patterns
   - Pottery or sculpting
   - Culinary tasks like decorating cakes
5. **Tasks with Soft or Deformable Objects**: These can be challenging for traditional robotic approaches due to the unpredictable nature of soft materials. Examples include:
   - Handling dough in a bakery setting
   - Manipulating soft fabrics or textiles
6. **Adaptive Gripping**: While there are algorithms for adaptive gripping, LfD can capture the nuances of how humans adapt their grip based on the object's material, shape, and weight. Examples include:
   - Handling fragile objects like eggs or glassware
   - Picking up objects that can deform, like filled bags
7. **Interaction with Humans**: Tasks where the robot needs to understand human intentions or needs based on gestures, motions, or other non-verbal cues. Examples include:
   - Personal care tasks for the elderly or disabled
   - Rehabilitative exercises where the robot assists a patient based on their movements
8. **Tasks Involving Feedback**: Tasks where tactile or other forms of feedback are crucial. Humans often adjust their actions based on the feel, which can be captured using LfD. Examples include:
   - Inserting a USB plug, where humans wiggle and adjust based on feedback
   - Opening a bottle cap where the force and twisting motion is adjusted based on feedback

Remember, the key advantage of LfD is its ability to capture the human demonstrator's adaptability and dexterity. However, the quality of the demonstration, the method used for capturing the demonstration, and the ability to generalize from the demonstration to new scenarios are all factors that will determine the success of LfD in a given task.



## Breakdown

**1. What LfD Provides:**

LfD is about learning the "how" of a task, not necessarily the "where." When you kinesthetically teach a robot to pick up objects, it learns the nuances of gripping, the forces to apply, the trajectories that are effective, etc. It doesn't inherently learn to detect or localize objects in arbitrary positions.

**2. Using LfD with MoveIt2:**

After training a model using LfD, you might integrate this model into a control pipeline that also uses MoveIt2. Here's a possible scenario:

- **Object Detection:** Use a camera and computer vision techniques (e.g., a deep learning-based object detector) to detect and localize the object in the robot's workspace.
- **Planning with MoveIt2:** Use MoveIt2 to plan a path to the object, ensuring that it doesn't collide with any obstacles.
- **Executing with LfD Model:** As the robot approaches the object, switch control to the model trained via LfD to perform the nuanced pick action, leveraging the "how" knowledge it gained from demonstrations.

**3. Generalization:**

The model's ability to generalize to objects in random locations primarily depends on the variability in the training data and the method used for learning:

- If during kinesthetic teaching you always pick an object from the exact same spot, then the model will be very specialized.
- If you trained with objects in various locations and orientations, the model will likely generalize better.

**4. Combining with Other Techniques:**

For the robot to adapt to objects in random positions, you typically combine LfD with other techniques:

- **Visual Servoing:** This is a method where the robot uses visual feedback (from a camera) to adjust its movements in real-time.
- **Feedback Control:** Use force/torque sensors or other feedback mechanisms to adjust the robot's movements based on the situation. For instance, if it feels an unexpected resistance, it can adapt accordingly.

**5. Real-world Expectations:**

- **Random Location with Known Objects:** If the robot has been trained with a variety of objects in diverse positions, it should be able to handle known objects in random locations given that it has a system (like computer vision) to detect the object's position.
- **Completely Unknown Objects:** If the object is entirely different from what the robot was trained on, success is more uncertain. The robot might still succeed if the task's general principles (gripping, trajectory) apply, but challenges can arise.

In essence, LfD teaches the robot the motions and actions required for a task based on human demonstrations. To enable a robot to work autonomously in varied environments, LfD is often just one part of a larger system that includes object detection, path planning (like MoveIt2), feedback control, and other components.
