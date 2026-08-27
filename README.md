# mvrt-hackathon-s26

### A presentation version of this document is available [here](https://docs.google.com/presentation/d/1eBOh-ol-1SGOnyzshVXi46T7th2BMpeOred9sAQ4vXk/edit?usp=sharing).
#### For the most up-to-date version of this document, click [here](https://docs.google.com/document/d/1Db60ImAYDRpjKwg5h1S7nhmssKlkxXsbTjxKP3F-6W8/edit?usp=sharing).

Why Team 68 Was Chosen:
* Easiest system to design and code (only had an arm and hand to work with)  
* Robot complexities translated to easy code/non-code complications  
  * Arm extending mechanism doesn’t need to be simulated since the “stack-like” arm extending is managed by the mechanism  
* Detailed videos on how the arms move and hand operates  
* Dev note: it was the first bot that had a detailed CAD to find real-ish values

Team 68’s 2023 Bot Details:
* Subsystems:  
  * Arm  
    * Extendable, there is one motor driving the wide chain  
      * Arm is shortest when the chain is most exposed (outside)  
      * Arm is longest when the chain is enveloped (inside the arm structure?)  
    * Rotatable; Can also rotate almost 180º (orthogonal to robot frame) and is controlled by two motors driving the thin chain  
  * Hand  
    * Driven by a single TalonFX motor connecting two separate belts for both fingers  
    * Outer most paddles are used to draw items inside and then is held by friction by inner wheels  
  * Drive  
    * Use the current implementation of Drive and Gyro  
  * Vision  
    * Seems to manage looking at April Tags around the map, the current implementation of Tracking may work \- need to test  
  * Tracking  
    * Tracks April Tags on the court, with 4 of them being at the scoring locations and human pickup point  
* Game tasks  
  * Gathering:  
    * Given a cone or cube when the match begins  
    * The rest of the cones/cubes are placed in the center area  
    * Players on one alliance can enter their own area and the center area, but not into the other alliance’s area  
    * The robot must be able to transport the cone/cube to their area and place it in the desired level  
  * Scoring:  
    * Robots must score (“charge up”) by placing the cone or cube on a desired level  
    * There are 3 levels (starts from ground level) of poles and flat surfaces, some require only cones or cubes, while others can require both  
    * Can earn points if the robot can climb onto their charge stations  
  * Final position:  
    * The robot should dock onto the charging station, must be flat  
  * First 15 seconds are autonomous, 2 min and 15 seconds are teleoperated  
  * Human players can provide game pieces to robots to score

Developer Experience & Feedback:
* Due to the lack of complex subsystems, the code for the robot remained minimal  
  * The new system did reduce some of the redundant code for recreating IO, IOSim, and IOReal classes  
* Comments on the new system:  
  * Abstracting away the Motor configs help reduce the code directly inside the subsystem (folder)  
  * Lack of documentation made it hard to pick up – needed to read the motor code  
    * JavaDoc goes a long way here, especially denoting the expected units – I had some trouble figuring out what ratio was needed for withSim() and withSensorToMechanismRatio() methods  
  * Substate system is inflexible at times – its usage is also unclear  
    * A value cannot be attached with a substate – for the arm, I can’t have it document it rotating and extending separately using the substate  
      * I could have had two different values and used substate but it would have meant it would only rotate or extend and not both at the same time  
    * Substates are used in the examples as further checks for a command and cannot be used as the basis for decision making  
      * In the superstructure, I used the substate as values to hold information (i.e. ARM\_ROTATE and (Score) LOW)  
      * This would go against the intended design but allowed for signaling of values (get from an array) without additional variables – values are stored in the enum itself (refer to SS.Score)  
    * Substates are wiped when the Command changes  
      * I found this behavior to be very frustrating when implementing manual control (just for testing purposes, provides a constant voltage to motor)  
      * In the end, I had to guard against the controller button being non-active and checking whether the current command is manual  
  * Flag system in Superstructure seems redundant  
    * I get why there are flags here, we can unify multiple different modes into one Command and/or substate(s) and have clearer logging  
    * This system seemed more of a hindrance – I opted to using Substate-Value enums instead to direct the value since using Flags often meant that you would lose data on what the user is specifically requesting unless you have extra field variables  
* I didn’t expect to spend this little time on this hackathon – it was hyped up to be something very important but with school priorities (looking at you APUSH\!), it seems quite minor in the grand scheme of things  
* Some feedback:  
  * I think more periodic check-ups with others are important – leads should actively review code and give feedback – I know you may have other commitments, so no pressure on implementing this  
  * Enforce stricter standards on code styling, documentation, and contributions to active repos – consider creating a CONTRIBUTORS.md file that outlines documentation and git usage  
  * Frontloading work – this may or may not be possible due to the scheduling of FRC events and outside factors, but smaller bits of work helps relieve stress and loosens deadlines  
  * A workboard – I often see leads asking other members directly for help, however there is no centralized place to see all the things we need to do as a division and as a team  
    * Maybe a part of the MVRT website (that does need a face lift) or a private organization Github repo to log todo items