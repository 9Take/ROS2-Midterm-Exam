<h1><b>ROS2 Midterm Exam</b></h1> 
<p style="font-size:16px;"> Test your understanding of <b>Publisher</b>, <b>Subscriber</b>, <b>Service</b>, and <b>Action</b> through TurtleBot3 Gazebo simulation. 
</p> <span style="color:blue"><b>Important Note: Use ROS2 Humble, Turtlebot3 Simulation : https://github.com/ROBOTIS-GIT/turtlebot3_simulations </b></span> 

<h1><b>Instructions</b></h1> <ul> <li>Use ROS2 Python or C++ (student choice)</li> <li>Each task will be tested in the TurtleBot3 Gazebo simulation.</li> <li>Code must be properly organized in a ROS2 package with clear node names.</li> <li>These exams are already separated into 3 parts (100 Points): <ul> <li>Section A: Publisher &amp; Subscriber (30 Points)</li> <li>Section B: Service (30 Points)</li> <li>Section C: Action (40 Points)</li> </ul> </li> <li>Students are allowed to use any tools, including generative AI like ChatGPT and Gemini, but students must clearly understand and be able to answer any questions that belong to the generated code.</li> </ul> 

<h3><b>Section A: Publisher &amp; Subscriber</b></h3> (30 points) 
  <p style="font-size:18px;"><b>Task A1 (Publisher)</b> – 10 pts</p> 
    <p> Create a node named “circle_publisher” that continuously publishes desired velocity commands (<code>geometry_msgs/msg/Twist</code>) to make Turtlebot3 move in a circle of radius ~0.5 meters. </p> 
    
  <p style="font-size:18px;"><b>Task A2 (Subscriber)</b> – 20 pts</p> 
    <p> Create a node named “odom_logger” that subscribes to “/odom” and prints: </p> 
      <ul> <li>Robot’s x, y position.</li> <li>Robot’s orientation (yaw angle).</li> </ul> 
      <p><b>Hint:</b> Please look up the keyword “quaternion to Euler angle”; the yaw angle is the heading angle of the robot.</p> 
    <p><b>Scoring:</b></p> <ul> 
      <li>Correct message type &amp; topic (5 pts)</li> 
      <li>Continuous publishing without errors (5 pts)</li> 
      <li>Proper extraction and display of odometry data (10 pts)</li> 
      <li>Code readability &amp; correct package structure (10 pts)</li> </ul> 

<h3><b>Section B: Service</b></h3> (30 points) 
  <p style="font-size:18px;"><b>Task B1 (Service Server)</b> – 20 pts</p> 
    <p> Create a service server node named “square_service_server” with service type <code>std_srvs/srv/Empty</code>. </p> <ul> <li>When called, the robot should move in a square path with 0.5 meter per side using velocity commands.</li> <li>After finishing, the robot should stop.</li> </ul> 

  <p style="font-size:18px;"><b>Task B2 (Service Client)</b> – 10 pts</p> 
    <p>Create a client node “square_service_client” to call your created service.</p> 
    <p><b>Scoring:</b></p> 
      <ul> <li>Correct service definition &amp; server (10 pts)</li> 
      <li>Correct client implementation (5 pts)</li> 
      <li>Robot executes a clear square trajectory (10 pts)</li> 
      <li>Stops at the end of the square (5 pts)</li> </ul> 
      
<h3><b>Section C: Action</b></h3> (40 points) 
  <p style="font-size:18px;"><b>Task C1 (Action Server)</b> – 25 pts</p> 
    <p> Create an action server node named “rotate_action_server” using a custom action definition <code>Rotate.action</code>: </p> 
      <img width="648" height="207" alt="image" src="https://github.com/user-attachments/assets/93446405-96aa-42ab-b59a-71b79a8645cc" /> 
    <p><b>The server should:</b></p> 
      <ul> <li>Rotate the Turtlebot3 in place by publishing to “/cmd_vel”.</li> 
      <li>Track the remaining angle of the robot and calculate the proper velocity through a simple P controller as follows:</li> </ul> 
      <img width="781" height="179" alt="image" src="https://github.com/user-attachments/assets/175d218a-d761-4e10-b77e-071cfbbe069d" /> 
      <ul> <li>Publish feedback every 0.1 second (10 Hz).</li> 
      <li>Stop and succeed when finished.</li> </ul> 
  <p style="font-size:18px;"><b>Task C2 (Action Client)</b> – 15 pts</p> 
    <p> Create a client node “rotate_action_client” that: </p> 
    <ul> <li>Sends a goal angle (e.g., +3.14 radians or 180 degrees).</li> 
    <li>Prints feedback (remaining angle).</li> 
    <li>Prints the result when done: “Goal reached successfully” or “Goal aborted”.</li> </ul> 
    <p><b>Scoring:</b></p> 
      <ul> <li>Action server implementation (10 pts)</li> 
      <li>Feedback &amp; result handling (10 pts)</li> 
      <li>Correct client implementation (10 pts)</li> 
      <li>Turtlebot3 reaches the target angle successfully with a tolerance of 10 degrees (10 pts)</li> </ul> 
      
<h3><b>Submission</b></h3> 
  <ul> <li>Students must upload the whole ROS2 package, including each node, as a compressed file into Google Classroom at the given time.</li> </ul>
