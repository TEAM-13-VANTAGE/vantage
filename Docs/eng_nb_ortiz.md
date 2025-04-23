
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

okay, I have forgotten slightly to update the engineering notebook here are a few updates:
the 
new semester started on the 10th of January and mostly consisted of breaking down what we needed to do for the rest of the semester in a presentation given by the teachers.

the following week (14th and 16th) consisted of checking requirements of the what we need to do or update that be the Jira backlog or having work done on the back log of the gazebo classic. We managed to implement one and two drone swarm.
Jim managed to get Lidar working I will ask him more about it tomorrow (today is the 20th). The other thing is getting a vm dedicated to crafting the current working position. Other than this I don't know what else we need to work on. Documentation definitely needs to be updated with the new requirements given.




21-24 jan

mostly worked on getting the gazebo drone moving. We got it to move on Thursday and back to trying to optimize it. Over the weekend I started drafting the new SRS section on my end. 

as far as 1/28th/2024 

I started researching a bit more on drone movement automation. I got sick this week I could not attend the scrum meeting.


looks like I forgot to update the engineering notebook! or it did not save great stuff! Okay summarizing a bit to rapid-fire things: SRS got updated and finished, WP updates and related.


I will update with this week primarily:
Tuesday,  February 11th: got to test WP and learned how to properly load and setup WP systems
they go something like: waypoint number, par1, par2, apr3, latitude(x), longitude(y), altitude?(z), comtinue to next WP? (0 or 1). the drones can now consistently get up and online and load the waypoints!
You arm, throttle, and make them takeoff on guided mode and then set them on auto. Issue that I am running into is the drone seem to not follow the waypoints.
Wednesday and Thursday updates! because updating this notebook after 8 hours of working is the next big thing I want to do! I have been tinkering and learned how to load the missions through the mav console even if we already learned last Tuesday how to load them on the terminal. I HAVE NO CLUE WHY IT IS MISSING THE WAYPOINTS! WOOOOOOOOOOOOOOOOOO! It seems that the errors or bugs it is running into might be between the following; WP radius which does not add up or maybe it does. (it is a 2-meter radius for the waypoint confirmation points I am testing WP with a 10m distance), potentially GPS signals?, I dunno what else? I am frustrated at the fact that there is not a lot of poking that has helped out on this work session. I tried setting up ROS as expected it made some amount of sense but without proper file setup it will not work as planned and without the drones properly following the WP I need to attack this problem from another  angle. hopefully, I come in with a fresh mind rather than a 4 am session or the sleep schedule not helping tomorrow. I want to figure this out so as not to hold back the team.

expect another update for Thursday later today at class time.

Decided to update on it Sunday the 17th: 
We decided to go back into the message module and we managed our first collision. It is easier to work with since the waypoint system was not working. Thankfully Jim realized I was testing in the wrong world file. for the weekend of 15-17th of February time was spent on the ROS research to get going for it. The mavros package is probably what we will use but it seems interesting to see how to apply it.

week of 2/20/2024 Tuesday to Thursday:
Tuesday was the end of the sprint focused on researching the load module message and met up with the product owner on Thursday. Thursday we planned out the rest of the semester or sprints. Mostly for those sprints for my load it is looking into SDD and configuring the world for gazebo implementation.




Ok I did forget to update this notebook in a minute but here is a summary of atleast the past 2 weeks 23 march-5th of april
Mostly it has been on trying to configure the virtual box machine. During this period of time we had a issue where ROS would not properly work leading the project to slightly have to backtrack into the 24.04 version of ubuntu. This lead to having to wait a bit until this was figured out in order to be able to further design the world simualtion for this project. While this is created and already made by the week of March 16th we needed to further improve it and double check it worked on the newer version.

esesntially we got the issue fixed recently and now we are almost done with the minimum viable product. This allows us to further try to improve the product in the little time we have left. for the past week I have researched a bit on how to get the values for the Gazebo sim I think we got it down?
I will wait on the meeting on the 8th and further report for it.


Thursday 10 and Friday the 11th: mostly coding and figuring a way to export the flioght pattern from gazebo to a bin file and from bin to CSV. This has also been the goal of the weekend a lot to work on still not really sure If i can get it to work in time.

Past weekend was working into the Bin to CSV files, we got it working the 14th and 15th. We just need t compare values and run parameter values.


The final week is upon us 4.22.2025: nothing really is happening a lot of submissions and well the presentation is thursday.



