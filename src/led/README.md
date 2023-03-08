# How to test
input 1 will open led; otherwise will stop led


To start the service run <br/>
`ros2 run led led` <br/> 
and 
<br/>
`ros2 service call /led/led std_srvs/srv/SetBool '{"data": 1}' `<br/> 
to test  
