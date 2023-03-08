# How to test
input 1 will trigger open door sound; otherwise will alarm

To start the service run <br/>
`ros2 run speaker speaker` <br/> 
and 
<br/>
`ros2 service call /speaker/speaker std_srvs/srv/SetBool '{"data": 1}' `<br/> 
to test  
