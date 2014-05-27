%{
    Before trying to run this example, please first compile the CRATES
    project, and then run the following from the command line:

    > roslaunch sim sw.launch world:=worlds/hawkshead.world

    This will start the ROS master and a simulation server, and bring up a
    GUI for the simulation. You are now ready to connect your cognitive
    controller client code to this simulation...

    REALLY IMPORTANT INFO ABOUT MESSAGES

    Much of the functionlity in the HAL is exposed over services, rather
    than topics. It turns out that services are just implemented as request
    response version of standard broadcast-based topics. Addressing is done
    by the topic namespace. So /hal/xyz/controller/Waypoint will only
    be 'heard' by entity 'xyz', even though another entity 'abc' may be
    listening for the same message types on /hal/abc/controller/Waypoint.

    In order for the CRATES custom messages to be exposed to the Malab ROS
    IO bridge, you'll need to dump the jar files created in the CRATES
    devel/share/maven/com/github/crates_msgs/xyz/a.b.c/xyz-a.b.c.jar dirs
    to %MATLABROOT%/toolbox/psp/rosmatlab/jars and run this in MATLAB:

    > rosmatlab_AddClassPath

%}

clear;

% Launch a ROS master on port 11311 on localhost.
%roscore = rosmatlab.roscore(11311);

% Create a new node named /mycontroller and connect it to the master.
node = rosmatlab.node('mycontroller',[],[],'rosIP','127.0.0.1');

% Add a publisher of a topic named /TOPIC to the node to send message of
%publisher = rosmatlab.publisher('/hal/hummingbird/Estimate','hal_quadrotor/State',node);

% Add a subscriber to a topic named /TOPIC to the node to receive message
subscriber = rosmatlab.subscriber('/hal/hummingbird/Estimate','hal_quadrotor/State',1,node);
subscriber.setOnNewMessageListeners({@rcvfn});

% Create a message



% Update the data field of the message and then publish the message
% iteratively.
for i = 1:10
    %publisher.publish(msg);
    pause(1)
end

% Remove the subscriber from the node.
node.removeSubscriber(subscriber);

% Delete the master.
%clear('roscore');

% Clean up
clear;

