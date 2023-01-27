The task was to make a chatroom for users who could view and send messages that all the users can see. A minimum of three nodes must be connected to this chatroom.

Approach Undertaken : I used the publisher - subscriber concept in ROS nodes. In my implementation, each node is a publisher as well as a subscriber to a single topic called "chat_topic". Nodes communicate via a custom message consisting of two string fields - one for holding the Username of the node and the other for the Message transmitted. All the nodes publish only via a single topic named 'chat_topic'.


Initially ros::spinOnce() was used but I replaced it with asyncspinner().

Reasons:
The standard ROSCPP spinner (ros::spin()) is single threaded, meaning it will only execute callbacks one by one, trying to respect the arrival date of messages or requests.

A callback will be executed only if the previous callback has finished. And the next callback will have to wait for the current callback to end.
On the other hand Async Spinner is a threaded spinner and callbacks will have it's own thread

AsyncSpinner is a threaded spinner. It means that each callback will get its own thread (if available).

spinOnce works on a single thread so for a node, it was not able to continuously keep refreshing the subscribed topic and update messages sent until the node published. Publishing messages and getting messages from subscribed topic was occuring at the same time. I wanted subscribed topic messages to continuously be updating by itself even if the node did not publish. Thus replaced spinOnce() with asyncspinner() which allowed the use of multiple threads for continuous updation of incoming messages.


Custom ROS Message used:
  
![Screenshot from 2023-01-26 23-58-33](https://user-images.githubusercontent.com/96124935/214928273-8b07e711-dacc-423d-9077-7c6d72c03664.png)

RQT Graph:![Screenshot from 2023-01-27 00-11-18](https://user-images.githubusercontent.com/96124935/214928303-ff83f7ab-b0c2-42c4-9579-eb9a151cb4e5.png)

YouTube Video Link: https://www.youtube.com/watch?v=hgdI8XGWiu8&ab_channel=AwesomeRoboticsAndTech
