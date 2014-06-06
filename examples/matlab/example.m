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

