The task was to make a chatroom for users who could view and send messages that all the users can see. A minimum of three nodes must be connected to this chatroom.


Approach Undertaken : I used the publisher - subscriber concept in ROS nodes. In my implementation, each node is a publisher as well as a subscriber to a single topic called "chat_topic". Nodes communicate via a custom message consisting of two string fields - one for holding the Username of the node and the other for the Message transmitted. All the nodes publish only via a single topic named 'chat_topic'.


Initially ros::spinOnce() was used but I replaced it with asyncspinner().


Reasons:
The standard ROSCPP spinner (ros::spin()) is single threaded, meaning it will only execute callbacks one by one.


On the other hand Async Spinner is a threaded spinner and callbacks will have it's own thread


spinOnce works on a single thread so for a node, it was not able to continuously keep refreshing the subscribed topic and update messages sent until the node published. Publishing messages and getting messages from subscribed topic was occuring at the same time. I wanted subscribed topic messages to continuously be updating by itself even if the node did not publish. Thus replaced spinOnce() with asyncspinner() which allowed the use of multiple threads for continuous updation of incoming messages.


Custom ROS Message used:
  



RQT Graph: