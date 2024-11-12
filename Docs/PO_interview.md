# Interview Questions

These questions are specifically for this semester’s scope, and not after the testing environment is completed.

What specific goals or problems are you looking to address with the  system?
 - Creating an **Open-Source** tool for aerial vehicle researchers 
    - there are many simulators, etc but no single source where you can test an autonomous vehicle end-to-end
    - two levels (high fidelity, low fidelity)
        - low fidelity: agent-based, don't care about sensors or any other "noise"
        - high fidelity: 3D simulation which is most commonly used in this type of research
            - Gazebo was chosen because it's commonly used
            - **High Priority** Modular "brain" functionality where you can plug in your "brain" to control the vehicle in the simulation
                - could be ArduPilot, very rule-based/algorithmic
                - could be neural network we train
                - Goal is for it to be modular and flexible
  - 
In terms of the scope of the system, is the system strictly a testing environment for UAS analysts?
 - It's a testing environment for anybody who wants to work with UAS simulations
    - Yes, the base goal for the Minimum Viable Product is to target UAS researchers/analysts.

Could you describe the range of collision scenarios that you want to be tested?
 - At first, we want the encounter scenarios that we already did to be done in our framework (Akbas can send them)

What UAVs are we expected to model and create simulations for? (Quadcopters, Single rotor, fixed wing, etc.)
 - For short term, whichever is easiest to get the simulator working. 
    - the simulations they did were a UAV against a crewed helicopter
    - whatever Gazebo has, we can take those parameters and create it in Julia
    - He wants a "walking skeleton" first and we can add more later - not high priority

How should the transition between low- and high-fidelity simulations be managed? Should any data be carried over between phases?
- I want you to make assumptions and very clearly/explicitly list them, beyond that it is up to you
    - in Low Fidelity, you have two points that are coming across each other and maybe they have perception ranges, etc. Not too many parameters
    - In Gazebo, you have a lot more sensors and parameters, so they're not one-to-one. 
    - It's okay if it's not a perfect mapping as long as you can make the assumptions relatively sensible and can list them clearly. 
    - As long as we can convert a low fidelity simulation to high fidelity, the rest is up to us. 
    - the reason we want Python is because they have other Python tools we can use to help us.
    - no requirement to run them both at the same time
    - Most likely, we will run a high number of low-fidelity simulations and we'll select a lower number of them in Gazebo
    - Run sims -> get results -> use them to determine what results we can get
    - moving skeleton is one type of encounter simulation with many variables of data, run a lot of variations in Julia, select some (~10) to run in Gazebo. 
    - Data transfer in JSON preferably

What other sensors and functions would you like to see implemented in the system (LiDAR, Drone Swarms, etc.)
 - We want everything, but LiDAR is top priority. (Do LiDAR first and worry about others later / low priority)

Would we need to create an AI model to train the simulations?
 - We need to make assumptions about how things work
    - We could make the "brain" class/whatever use the parameters for making the JSON format
        - "brain" outputs simulation data in JSON
        - JuliaSIM and Gazebo interface have the functionality to decode the JSON for their respective fidelity level
    - There are known right-of-way rules that we could think about too

What level of customization do you expect users to have in setting up or modifying test scenarios?
 - He cares about the main parameters (distance, speed, etc) related to decision making
    - don't worry about physical environment (rain, etc) in the beginning. 

Let’s say that you are using the system, what would you like to see in the GUI in terms of functionality?
 - He doesn't care about the GUI, he would like to run simulations without any visualization when he wants
    - if there's a way to run Gazebo without rendering, we should try to use it

Are there specific benchmarks or standards that these tests must meet to ensure validation?
 - If we are talking about the test that the users should be able to do, they should be able to set the standards/benchmarks

Are there any questions we missed or comments you would like to add in terms of this project?
 - For the rest of this semester, focus on getting one (rough) scenario running in both simulators first
    - if we can get it working, try to be able to control it using a common interface
        - he doesn't care how good it looks, it could just be Python code API.
        - once we have Gazebo working, Julia working, we can control the simulations, we can start going into more detail

We are considering scalability, modularity, and compatibility in terms of future proofing the product. Understanding that Gazebo was used previously for the high-fidelity simulation, would we be able to explore other options such as Unity for this? There seems to be more documentation with regards to Unity and ROS2 being compatible compared to Gazebo. It is understandable that we have about a month into the semester, but we have been struggling to set up Gazebo due to having little or poor documentation about how to set it up with a higher version of Ubuntu and ROS2.
 - The advantage of gazebo is this: ROS integration makes it easier to use it with real hardware
 - If we can get Gazebo working, we can get it to run on real drones
 - Gazebo is preferred because it's more specialized for this use case
 - There's nothing wrong with Unity. If this was something like a personal project, it would be okay but for an Open-Source system, people need to use it. 
    - Unity isn't open source
    - If it comes to a point where Gazebo is such a big bottleneck, we can go to other solutions
        - Since other people are using it, he's hoping it's not 

for the software architecture diagram in the SDD are we supposed to model it based off the preexisting code that you made, our future code, or both
 - Model it on what we have done before, but if there are any issues we can improve it

How will data be exchanged between Gazebo and Julia, and what is the expected data flow structure?
 - JSON is preferable

How are we managing timestamp synchronization across Gazebo and Julia?
 - Not needed

What specific data points are we logging, and how much data volume do we expect?
 - Essential data for decision making like speed/distance etc

What specific parameters are required for data exchange, and are these parameters standardized across simulators?
 - Decision making-related parameters that don't depend on sensors 
    - detection distance (a vehicle is coming and DUT detects that vehicle)
        - in Gazebo, the sensors could change this / some noise that we don't care about 
    - speed, position, acceleration
    - dangerous distances
    - starting locations
    - from a bird's eye view, both sims should look the same. 
        - One might be just points and the other modeled drones and sensors, but they should be essentially the same
    - You have Julia simulation which creates a JSON file and sends it to main controller
    - other simulator gets it, Julia fills in a portion of JSON and fills the rest with default parameters
    - Gazebo sim can tweak the more detailed parameters
    - "brain" would be easier to implement if we have separate ones in Gazebo and Julia, but a common one would be ideal
        - it should be connected to whatever is running

How will we validate that the simulation outputs from Gazebo and Julia are accurate and synchronized?
 - They don't have to be synchronized fully because the results might be different
 - as long as the initial parameters are the same, it will be ok
 - the simulation validation is a separate problem
 - If we do the interface right (the correct parameters are transferred from one sim to the other), we will get correct results

if python is preferable to be used for the controller what was the reason Julia was preferred for the low fidelity sim?
- Julia is preferred because there is already work in it (Akbas says it's the best choice for agent-based simulations) and Python is preferred because it's commonly used and would be easy for other people to modify/use. 

does the controller need to have a GUI
- not necessary, but nice to have (not a priority)

- Akbas is excited about the project, he hopes we can get some drones flying so we can get excited about it 