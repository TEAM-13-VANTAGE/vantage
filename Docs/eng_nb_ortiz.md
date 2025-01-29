
Small summary on sprint 1
Team and github repository was created. The applications to be used
where downloadded in to the devices. THis includes linux, julia, gazebo, and ardupilot.
 
Tested that the installed apps work properly. I created a small Julia "Cheatsheet" the type you would use to remember or have examples of how to code basics things in a programmihng language in order to be able to reference it for quicker coding pace.
Julia seems to be a rahter big amalgam of coding languages?, You can cross reference syntax similar to java and c in all the areas.

October 1, 2024    
Started sprint 2 met with the team. Started the Julia installation document.

October 2, 2024     
Finished julia installing file. Most of the information for it is on the julia website making it easy to access the document. I checked out the new UAW-Testing github repository in order to be able to familiarize myself with the files a bit morel. It will take a bit more time in order to fullly dive into it.

A bit of weird news? Sprint 2:
hurricane hit and classes got cancelled from OCtober 8th to the 18th. I lost power at the apartment making it rather difficult to work over the week.


October 15, 2024

new study problem to investigate: how do you use julia to communicate with ardupilot?

Use ROS to as a middle man for julia and ardupilot.

goals for the day:
1) install ROS in the VM (done)
2) document the ROS installation
3) create a basic cheatsheet for ROS?
4) maybe start the process of the development of the ROS integration?



October 20, 2024
A bit delayed but the documentation for the installation has been created. I managed to investigate and document the installation. The ROS version to be usaed is ROS FOXY.

updating the goal list for the upcoming week:
1)document the ROSfoxy installation process (Done)
2)start the integration process of julia and ros
3)start the integraation of ROS and Gazebo
4)start the new test process in for the collision test






October 22, 2024

researching how Gazzebo works with ROS to progress that part.

FOund this video for that before class:
https://www.youtube.com/watch?v=laWn7_cj434&ab_channel=ArticulatedRobotics


Appart fromthis the team met up and refactored to see what to work.


here are a few things to update teh way to work moving forward:

I need to understand how both simulators work, so I started working on a shell script to run both simulators. 
instead of running it in ros if we run it in shell script we can get teh same deal for it.

October 23, 2024

Okay, the code shell for the shell script is almost done I need to try to get it running 9.14 pm will keep updating on it.


October 24,2024

The team met today, but the dependency issues are still at hand. I tried to see if I could run the Julia simulator. It has been giving an error of not finding a file, which is weird because the file does exist. I have peered into the issue today for a few hours, but to no avail yet.


October 26, 2024 
the team started working on the SDS document.


October 27, 2024


I returned to try to test out the Julia programs, but the same issue persisted. There are deprecation notices on some packages. Okay, slight update: Some code is running in Linux Julia but not in VS code. I will run through the code tomorrow to check and see what exactly is hitting the errors. This way, I can finally work on the low-fidelity simulator.


October 28, 2024

More SRS documentation: chapter for req worked on.
need to check up on updating this.
I might need to ask about what requirements are in our project scope or what was in previous project scopes.


October 29, 2024

The team met up and uploaded the SRS clearing a few questions in class.


October 31, 2024

Regular group meeting, the SDD workload was divided and re-evaluated the way we worked.

November 3, 2024

Worked in SDD part 5.
November 4, 20244

During the interview with Dr. Akbas, many points were made that should help guide the direction and development of the project for the rest of the semester.

November 5, 2024
The VAntage group meet to finalize the SDD document.  


NOV 8 and 12th, 2024
Presentations ran in class.


A brief update on NOV 14, 2024

Meetup and updated the teacher

Nov 18th, 2024
Worked a bit on the SDD

Nov20, 2024
Abit more work on SDD
November 26, 2024
readfeedback for SRS and meet with Product owner talked about future goals and whata is to be done for the rest of the semester.

November 28 no class

12/1/2024

trying to get LIDAR to work. running through some debug of Ardupilot sitl. Could not get it online

12/3/2024

Started 10min presentation work, testing document and last meetup in the classroom

12/4/2024

presentation recording finished, more work on testing document


12/5/2024

The final presentation is today. Poster presentation we will see how it goes.

okay I have forgotten sllightly to update the engineering notebook here are a few updates:

new semester started the 10th of january mostly consisted on breaking down what we neeeded to do for the rest of the semesters in a presentation
given by the teachers.

the following week (14th and 16th) consisted on checking requirements of the what we need to do or update that be the Jira backlog or having work done on the back log of the gazebo classic. We managed to implement one and two drone swarm.
Jim managed to get Lidar working I will ask him more about it tomorrow (today is the 20th). The other thing is getting a vm dedicated to crafting the current working position. Other than this I don't know what else we need to work on. Documentation definitely needs to be updated with the new requirements given.




21-24 jan

mostly workde on getting the gazebo drone moving. We got it to move on Thursday and back to trying to optimize it. Over the weekend I started drafting the new SRS section on my end. 

as far of 1/28th/2024 

I started researching a bit more on the drone movement automization. i got sick this week I could not attend the scrum meeting.

