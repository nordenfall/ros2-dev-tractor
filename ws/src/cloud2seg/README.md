📦 cloud2seg — LiDAR→Camera Projection & 3D Obstacle Extraction

ROS2 Humble package for point cloud projection, segmentation-based object detection, and 3D bounding box generation.

🧭 Описание

cloud2seg — это ROS2-пакет, который:

синхронизирует камеру, лидар, CameraInfo и ID-сегментацию;

проецирует точки лидара в изображение камеры;

извлекает объекты по ID-сегментации;

формирует их 3D-границы (bounding boxes);

публикует список препятствий в формате Float32MultiArray.

Пакет работает:

с bag-файлами (use_sim_time = true),

в реальном времени с настоящими сенсорами,

с TF или без TF (ручные экстринсики),

в режиме debug overlay или полностью headless.

🗂 Структура проекта
cloud2seg/
├── cloud2seg/
│   ├── __init__.py
│   └── pcd_projector_node.py
├── launch/
│   └── project_bag.launch.py
├── package.xml
├── setup.cfg
└── setup.py

🚀 Возможности
✔ Синхронизация четырёх источников

Image

CameraInfo

PointCloud2

Segmentation ID (mono8/mono16)

Через ApproximateTimeSynchronizer.

✔ Проекция точек лидара в изображение

Формулы:

𝑋
𝑐
=
𝑅
⋅
𝑋
𝑙
+
𝑡
X
c
	​

=R⋅X
l
	​

+t

проекция:

(
𝑢
,
𝑣
)
=
Π
(
𝐾
⋅
𝑋
𝑐
)
(u,v)=Π(K⋅X
c
	​

)

Используются:

интринсики (K, D) из CameraInfo,

экстринсики (R, t) из TF или вручную.

✔ Детекция препятствий

По каждому ID-классу создаётся 3D-bouding box:

center = (min_xyz + max_xyz) / 2
size   = (max_xyz - min_xyz)


Каждый объект описывается 7 значениями:

x, y, z, size_x, size_y, size_z, class_id

✔ Публикация результатов

Топик:

/obstacles_3d


Тип:

std_msgs/Float32MultiArray


Формат данных:

[x1,y1,z1,sx1,sy1,sz1,cls1,
 x2,y2,z2,sx2,sy2,sz2,cls2,
 ... ]

⚙️ Установка
cd ~/ws/src
git clone <repo> cloud2seg
cd ~/ws
colcon build --packages-select cloud2seg
source install/setup.bash

▶️ Запуск с bag-файлами
ros2 launch cloud2seg project_bag.launch.py \
  out_dir:=/path/to/out \
  use_sim_time:=true

▶️ Запуск в реальном времени

В launch:

'use_sim_time': false


И никакого /clock — это realtime mode.

📡 Параметры пакета
Параметр	Тип	Описание
use_sim_time	bool	Использовать время bag-файла. Для live mode — false.
image_topic	string	Топик изображения.
caminfo_topic	string	Топик CameraInfo (интринсики).
cloud_topic	string	Топик лидара.
seg_topic	string	Топик ID-сегментации.
camera_frame	string	frame камеры в TF.
lidar_frame	string	frame лидара.
zmin	float	Минимальная глубина Z.
zmax	float	Максимальная глубина Z.
rmax	float	Ограничение по дальности (0 = выключено).
thick	int	Толщина точек в debug overlay.
publish_overlay	bool	Публиковать наложение точек на изображение.
🟪 Работа без TF (/tf и /tf_static)

Если никто не публикует экстринсики (lidar → camera), можно:

Вариант 1 — добавить статический TF в launch
Node(
  package='tf2_ros',
  executable='static_transform_publisher',
  arguments=['x','y','z','roll','pitch','yaw','camera','lidar']
)

Вариант 2 — захардкодить R,t прямо в коде

Удалить:

tf = lookup_transform(...)
R = quat_to_rot(...)
t = ...


Добавить:

R = np.array([[...],[...],[...]])
t = np.array([[tx],[ty],[tz]])


И использовать:

Xc = (R @ P.T) + t

🟦 Работа без CameraInfo (ручные интринсики)

Удаляем:

K = np.array(cam_msg.k).reshape(3,3)
D = np.array(cam_msg.d)


Заменяем на:

K = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0,  0,  1],
])
D = np.array([k1,k2,p1,p2,k3])  # или zeros(...)

🧪 Типичные проблемы и решения
Ошибка	Причина	Решение
No TF data	нет экстринсиков	дать TF или вручную задать R,t.
CameraInfo empty	YAML не загружен	задать intrinsics вручную.
ATS: no synchronized messages	неправильное время	включить/выключить use_sim_time.
projectPoints(): bad pts	K или D неверные	проверить CameraInfo.