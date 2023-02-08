# How to test
input 1 will lock the door; otherwise will unlock


To start the service run <br/>
`ros2 run door_lock door_lock` <br/> 
and 
<br/>
`<br/> service call /lock_service/lock std_srvs/srv/SetBool '{"data": 1}' `<br/> 
to test  
