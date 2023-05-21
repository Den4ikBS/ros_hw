# HW Intro to ROS

В домашней работе было необходимо в ROS создать пакет, в состав которого входит один узел для симуляции робота с дифференциальным приводом. Управление симулятором робота осуществлено с помощью пакета turtlebot3_teleop. С помощью RViz отрисовывается траектория робота. Робот-симулятор является программой, разработанной для эмуляции движения робота на основе команд, получаемых через интерфейс ROS (Robot Operating System). Программа моделирует двигатели робота и оценивает положение и скорость на основе данных об энкодерах.
Домашнее задание выполнили студенты группы КРБО-01-20: Гордеев Д.В., Губернев А.Ю., Опалев М.С.

## Зависимости

Для работы робот-симулятора необходимо наличие следующих зависимостей:

- ROS (Robot Operating System): Официальная система операционных функций для роботов. Для установки ROS следуйте официальной документации, доступной на сайте [ROS](http://www.ros.org/).

Библиотеки и пакеты ROS:

- `control`: Библиотека для управления системами и обработки сигналов.
- `std_msgs`: Стандартные сообщения ROS.
- `geometry_msgs`: Сообщения ROS для работы с геометрией и преобразованиями.
- `nav_msgs`: Сообщения ROS для работы с навигацией и оценкой положения.
- `robot_simulation`: Пакет с пользовательскими сообщениями ROS, используемыми в проекте.



## Установка и запуск

Для установки и запуска робот-симулятора выполните следующие шаги:

1. Установите ROS на вашу систему, следуя инструкциям, предоставленным на официальном сайте [ROS](http://www.ros.org/) или же следуя [инструкциям по установке Ubuntu и ROS](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

2. Склонируйте репозиторий с кодом робот-симулятора и turtlebot3_teleop в catkin_ws/src на вашу локальную машину:
git clone https://github.com//Den4ikBS/ros_hw
git clone https://github.com/ROBOTIS-GIT/turtlebot3

3. Установите все необходимые зависимости. Если вы используете `apt` для управления пакетами ROS, выполните следующую команду:

```sh 
sudo apt install ros-noetic-control ros-noetic-std-msgs ros-noetic-geometry-msgs ros-noetic-nav-msgs
```

4. Запустите ядро ROS.
```sh
roscore
```

5. Далее в новом терминале соберите пакеты ROS с помощью инструмента сборки `catkin_make`. Перейдите в корневую папку вашего рабочего пространства Catkin и выполните команду:
```sh
catkin_make
```

6. Запустите робот-симулятор, используя команду `rosrun` и указав путь к исполняемому файлу `robot_simulator.py`:
```sh
rosrun robot_simulation robot_simulator.py
```
7. Включение `turtlebot3_teleop` для считывания с клавиатуры. Необходимо в новом терминале ввести:
```sh
export TURTLEBOT3_MODEL=waffle; roslaunch turtlebot3_teleop  turtlebot3_teleop_key.launch
```
8. Для визуалиции необходимо включить RVIZ. Для этого в новом терминале выполнить:
```sh
rosrun rviz rviz
```
Теперь робот-симулятор готов к работе!
## Как он работает
Робот-симулятор подписывается на топик `/cmd_vel` для получения команд на управление движением. Когда поступает новая команда `/cmd_vel`, программа эмулирует движение робота на основе этой команды. Данные о положении и энкодерах робота публикуются на топиках `/enc` и `/odom` соответственно.
Сначала было создано 2 паблишера. Первый публикует показания энкодеров с левого и правого двигателя. Второй паблишер отправляет показания одометрии, одни из которых: координаты робота (x,y), линейная скорость по оси x,y, угловая скорость по оси z и ориентация робота, выраженная кватернионом.
```sh
enc_pub = rospy.Publisher("/enc", Encoder, queue_size=10)
od_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
# публикует показания левого и правого энкодеров
def talker_enc(l_enc, r_enc):
    msg = Encoder()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "enc"
    msg.left_encoder=l_enc 
    msg.right_encoder=r_enc 
    rospy.loginfo(msg)
    enc_pub.publish(msg)

def talker_odom(pose, twist):
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "odom"
    msg.pose = pose 
    msg.twist = twist 
    rospy.loginfo(msg)
    od_pub.publish(msg) 
```
Класс mot_mod описывает один из двигателей мобильного робота. Изменение угловой скорости для колеса симулируем апериодическим звеном с помощью библиотек control и tf. Передаточная функция имеет коэффициент усиления k = 1и постоянная времени T = 0.1. Для каждого колеса есть переменная enc, которая хранит в себе показания энкодера. В методе step происходит вычисление текущей угловой скорости колеса, а также изменение показания энкодеров. В параметры мы передаем нужную нам угловую скорость и момент времени, в которым происходит вычисление.
```sh
class mot_mod:
    def __init__(self):
        k = 1
        T = 0.1
        W = control.tf(k, [T, 1])
        self.sys = control.LinearIOSystem(W)
        self.x = [[0, 0]]
        self.w_target = 0
        self.enc = 0

    def step(self, w_target, dt, cur_t, prev_t):
        a, w, self.x = control.input_output_response(self.sys, [prev_t, cur_t], [self.w_target, w_target], self.x[0][1], return_x=True)
        # разница между показаниями энкодера на текущей и предыдущей итерациях 
        d_enc = int(w[1] * dt / 2 / 3.1415926535 * 4096)
        
        # угловая скрорость колеса на данный момент
        cur_w = d_enc * 2 * 3.1415926535 / dt / 4096

        self.enc += d_enc
        self.w_target = w_target
        return cur_w
```
Следующий класс motion_control() связан с управляющими командами. Внутри создаем подписчика, который подписывается на управляющие команды, а когда эти команды приходят вызывается метод callback( о нём далее). Также здесь создаем переменные для хранения данных, которые потом отправятся в одометрию, а также задаем расстояние между колесами и размеры самих колес. 
```sh
class motion_control:
    def __init__(self):
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.callback) 
        self.pose = PoseWithCovariance()
        self.twist = TwistWithCovariance()
        self.l_mot = mot_mod()
        self.r_mot = mot_mod()
        self.rot = 0
        self.L = 0.287
        self.r = 0.033
        self.prev_t = rospy.get_time()
```
В методе callback() происходят математические вычисления, связанные с линейной скоростью робота, угловой скоростью робота, изменение его координат и ориентации. 
```sh
    def callback(self, msg):
        rospy.loginfo("Linear Component: [%f]"%(msg.linear.x))
        rospy.loginfo("Angular Component: [%f]"%(msg.angular.z))
        V = msg.linear.x 
        Om = msg.angular.z
        # Найдем требуемые угловые скорости колес
        lw = (V - 0.5*Om*self.L)/self.r
        rw = (V + 0.5*Om*self.L)/self.r
        cur_t = rospy.get_time()
        dt = cur_t - self.prev_t
        # Вычислим текущие угловые скорости колес
        cur_spd_lw = self.l_mot.step(lw, dt, cur_t, self.prev_t)
        cur_spd_rw = self.r_mot.step(rw, dt, cur_t, self.prev_t)
        # Вычислим текущую линейную и угловую скорости робота, а также изменение его координаты и угол поворота
        V = 0.5*self.r * (cur_spd_lw + cur_spd_rw)
        Om = self.r/self.L * (cur_spd_rw - cur_spd_lw)
        x = V * cos(self.rot) * dt
        y = V * sin(self.rot) * dt
        self.rot += Om*dt
        self.pose.pose.position.x += x
        self.pose.pose.position.y += y
        q = quaternion_from_euler(0, 0, self.rot)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

        self.twist.twist.linear.x = V * cos(self.rot)
        self.twist.twist.linear.y = V * sin(self.rot)
        self.twist.twist.angular.z = Om
        self.prev_t = cur_t
```
В main() выполняется основной цикл программы
После запуска rviz в левом верхнем углу в параметрах Global Options-> Fixed Frame нужно написать odom.
После этого в левом нижнем углу нажать на кнопку "Add"->Odometry. В параметре Topic выбираем /odom.
В результате получаем:![Окно RVIZ с отображенным роботом](/result.jpg)

## Как управлять
С помощью кнопок на клавиатуре можно управлять линейными и угловыми скоростями робота, а также останавливать его: с помощью кнопки А можно прибавлять, а с помощью кнопки D убавлять угловую скорость робота, кнопкой W прибавлять, а кнопкой X убавлять линейную скорость робота. Кнопка S останавливает робота.

## Дополнительная информация

- Для получения более подробной информации о работе робот-симулятора обратитесь к комментариям в коде `robot_simulator.py`.

