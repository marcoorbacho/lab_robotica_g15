docker rm lab_rob_container

docker run -it ^
--env="DISPLAY=host.docker.internal:0" ^
--name lab_rob_container ^
--net=host ^
--privileged ^
--mount type=bind,source="C:\Users\Marco Corbacho\lab_rob_shared",target=/home/MarcoCorbacho/lab_rob_shared ^
lab_rob_image ^
bash

:: COMANDO PARA PODER EJECUTAR 2 CONTENEDORES O MAS A LA VEZ EN OTRO TERMINAL
:: docker exec -it lab_rob_container bash

:: primero dentro de catkin_ws hacer source devel/setup.bash y catkin_make, si no funciona, hacerlo alreves LANZAR ROSCORE ANTES