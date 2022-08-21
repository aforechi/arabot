# Arabot - An arm bot mounted on a 2WD robot car

- Tutorial de instalacão do ROS no Arduino <http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup>

- Detalhes sobre o pacote rosserial <http://wiki.ros.org/rosserial>
```
sudo chmod 666 /dev/ttySx
sudo cu -l /dev/ttySx
```

- Tutorial de criacão do projeto cmake para arduino <http://wiki.ros.org/rosserial_arduino/Tutorials/CMake>

- Tutorial de instalação do simulador da base <https://github.com/duckietown/duckietown-sim-server>
```
catkin config -DARDUINO_SDK_PATH=/Applications/Arduino.app/Contents/Resources/Java
```
## Compilar

1. Abra o terminal do linux e execute os comandos abaixo somente se for preciso:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/methylDragon/teleop_twist_keyboard_cpp.git
```
2. Descompacte o arquivo em anexo e copie a pasta arabot para o diretorio src criado acima
3. Confira se o arquivo launch está configurado com porta correta do Arduino
```
nano src/arabot/launch/arabot.launch
```
4. Configure os pinos dos motores no sketch do arduino localizado na pasta arabot/src/arabot_ros
5. Finalize a implementação do programa arabot/src/arabot/src/armbot.cpp para poder digitar valores entre 0 e 179 para cada um dos servo motores.
