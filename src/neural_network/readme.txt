Created a new ROS message type and added the coresponding lines in CMakeLists.txt

For ROS Hydro and later, you need to uncomment these lines: 
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

In earlier versions, you may just need to uncomment one line: 

generate_messages()
