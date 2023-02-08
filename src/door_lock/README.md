# How to test
input 1 will lock the door; otherwise will unlock


To start the service run <br/>
`ros2 run door_lock door_lock` <br/> 
and 
ros2 service call /lock_service/lock std_srvs/srv/SetBool '{"data": 1}' to test
