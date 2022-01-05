# tb3-ros2-telegram-high-level-controller
A telegram based ROS2 high level controller for Turtlebot3. 


The program consists of 3 part which are
- telegram_pub to get the latest message and publish into a topic
- high level controller based on the telegram topic published topic
- launch file to launch both 2 nodes and turtlebot3 gazebo ros2


## Node Red Server
A node-red server used as the bridge between HTTP and HTTPS requests. The server being used is http://node-server-dummy.herokuapp.com/red

Endpoint to get the last message is "/lastmessage" returning data in format "<chat_id>;<text>"

## Telegram Bot 
Link of the bot is [here](https://t.me/ros2_tb3_bot)

##  Credits
- nicolasaw [high-controller-tb3-ros2](https://github.com/nicolasaw/high-controller-tb3-ros2) repo.
- yhorose [cpp-httplib](https://github.com/yhirose/cpp-httplib) repo.
