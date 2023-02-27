
Task Description:
Autonomous Traversal of Four Wheeled Bot from one GPS coordinate to another GPS coordinate.

A single roscpp traversal node is used wherein using the gps data, distance to end destination is calculated using haversine formula. IMU data is used for calculating the current orientation and then I calculated the required heading to rotate to. A proportional controller is implemented based on the angular difference and linear distance to destination coordinates

rqtgraph:

![Screenshot from 2023-02-24 20-56-26](https://user-images.githubusercontent.com/96124935/221219255-bf6888cf-b80c-44fa-aaa8-742839b9716b.png)

rostopics:

![Screenshot from 2023-02-24 20-57-28](https://user-images.githubusercontent.com/96124935/221219338-1cfa2f70-4279-49ca-8f81-e6946bbe22df.png)

Rosmsgs used:

![Screenshot from 2023-02-27 19-25-44](https://user-images.githubusercontent.com/96124935/221584013-ab80217b-aeea-4eb3-940e-8d3fb9dc76ca.png)

![Screenshot from 2023-02-27 19-26-22](https://user-images.githubusercontent.com/96124935/221584019-5f41443d-181c-45f3-928f-2c46558d912b.png)
![Screenshot from 2023-02-27 19-27-46](https://user-images.githubusercontent.com/96124935/221584030-268cf7d3-cf51-442a-b158-e923248c188a.png)
![Screenshot from 2023-02-24 20-57-28](https://user-images.githubusercontent.com/96124935/221583944-17e260b8-9f6b-43c0-b527-d60ee4bb48ce.png)

Link to YouTube Video:
https://www.youtube.com/watch?v=acHFoFqlU_A&ab_channel=VyshnavDavanagere
