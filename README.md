# up_week1: Under Pressure AUV Team, 1. Hafta Yazılım Ödevi
 
Gazebo Fortress simülasyon sahnesi, sabit kamera görüntüsünün ROS 2 tarafına aktarılması ve iki ayrı ROS 2 düğümü (heartbeat publisher / subscriber) ile bir kamera izleme düğümü (camera_monitor).
 
**Aday:** Idil Sincer
 
## 1. Ortam ve sürümler
 
| Bileşen | Sürüm |
|---|---|
| İşletim sistemi | Ubuntu 22.04 LTS (sanal makine) |
| ROS 2 | Humble Hawksbill |
| Simülasyon | Gazebo Fortress (`ign gazebo`) |
| Köprü | `ros_ign_bridge` (Humble'da `ros_gz_bridge`'e yönlendirilir) |
| Dil | Python 3.10 |
| Derleme | colcon, `ament_python` |
 
## 2. Bağımlılıklar
 
```bash
sudo apt update
sudo apt install ros-humble-desktop \
  ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo ros-humble-ros-ign-image \
  ros-humble-rqt-image-view python3-colcon-common-extensions
```
 
`package.xml` içindeki çalışma zamanı bağımlılıkları: `rclpy`, `std_msgs`, `sensor_msgs`, `ros_ign_bridge`, `ros_ign_gazebo`, `launch_ros`, `ament_index_python`.
 
## 3. Derleme ve çalıştırma
 
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:idilsincer/ros2-gazebo-fortress-up-test-world.git up_week1
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch up_week1 week1.launch.py
```
 
Tek launch dosyası Gazebo dünyasını (`-r` ile doğrudan çalışır halde), kamera köprüsünü ve üç düğümü başlatır. Dosya yolları paketin kurulu `share` dizininden bulunur, kullanıcıya özgü yol yoktur.
 
Kamera görüntüsünü görmek için launch çalışırken ayrı bir terminalde:
 
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```
 
Sanal makinede kamera render'ı çalışmazsa: `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch up_week1 week1.launch.py`
 
## 4. Paket yapısı
 
```
up_week1/
├── launch/week1.launch.py
├── up_week1/
│   ├── camera_monitor.py
│   ├── heartbeat_publisher.py
│   └── heartbeat_subscriber.py
├── worlds/test_world.sdf
├── package.xml
├── setup.py
└── README.md
```
 
## 5. Düğümler, topic'ler ve mesaj tipleri
 
| Düğüm | Görev | Topic | Mesaj tipi | Yön |
|---|---|---|---|---|
| `heartbeat_publisher` | Saniyede bir, adayın adını ve artan sıra numarasını içeren mesaj yayınlar (`Idil Sincer - heartbeat #N`) | `/up/heartbeat` | `std_msgs/msg/String` | broadcast |
| `heartbeat_subscriber` | Heartbeat mesajlarını alır ve terminale yazdırır | `/up/heartbeat` | `std_msgs/msg/String` | listener |
| `camera_monitor` | Kamera görüntüsüne abone olur, yaklaşık saniyede bir genişlik, yükseklik, encoding ve toplam alınan mesaj sayısını raporlar | `/camera/image_raw` | `sensor_msgs/msg/Image` | listener |
| `parameter_bridge` (hazır `ros_ign_bridge` aracı) | Gazebo kamera görüntüsünü ve saati ROS 2'ye aktarır | `/camera/image_raw`, `/clock` | `sensor_msgs/msg/Image`, `rosgraph_msgs/msg/Clock` | Gazebo → ROS |
 
`camera_monitor`, kamera akışı için `qos_profile_sensor_data` (best effort) kullanır. Aktarım kesilirse sayaç artmaz, aktarım dönünce kaldığı yerden sayar.
 
## 6. Simülasyon dünyası
 
Dünya adı `up_test_world`, dosya `worlds/test_world.sdf`. Konumlar metre, açılar radyan cinsindendir (SDF varsayılanı).
 
| Model | Geometri | Konum (x, y, z) | Boyut | Durum |
|---|---|---|---|---|
| `ground` | kutu (zemin) | 0.0266, 0.0264, -0.0351 | 12 × 12 × 0.05 m | sabit |
| `box` | kutu | 1.858, -0.298, 0.5 | 1 × 1 × 1 m | sabit |
| `capsule_1` | kapsül | -5.096, 4.218, 0.5 | r = 0.2 m, uzunluk 0.6 m | sabit |
| `static_cylinder` | silindir | -1.0, 1.0, 0.5 | r = 0.3 m, uzunluk 1.0 m | sabit |
| `capsule_0` (**kırmızı hedef**) | kapsül | 1.856, -1.618, 0.490 | r = 0.2 m, uzunluk 0.6 m | sabit |
| `fixed_camera` | kamera | 0.9, -4.5, 1.5 (roll 0, pitch 0.265, yaw 1.305) | 640 × 480, R8G8B8, 10 Hz, yatay FOV 1.047 rad | sabit, araca bağlı değil |
 
Işık: `directionallight` (yönlü ışık, gölgeli). Tüm nesnelerin görsel ve çarpışma geometrileri tanımlıdır. Kamera bir kırmızı hedefi ve kutuyu görüş alanında tutar. Kamera modelinde bir link vardır ve `Sensors` sistem plugin'i (`ogre2`) dünya dosyasında yüklüdür.
 
## 7. Kaynaklar ve yapay zekâ desteği
- https://gazebosim.org/docs/latest/ros2_integration/
- https://gazebosim.org/docs/latest/ros_installation/
- https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy
- https://blog.gistre.epita.fr/posts/marie.floissac-duforez-2023-09-27-introduction-to-ros2-humble-and-gazebo-fortress/
- https://www.youtube.com/watch?v=YV8hlpBOhtw
- https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy
- https://www.youtube.com/watch?v=O0729K-7VEY&list=PLNw2RD-1J5YYvFGiMafRD_axHrBUGvuIg
- https://reference-guide-cb79e5.pages.oit.duke.edu/gazebo-fortress-help/
- **Yapay zekâ desteği:** Yapay Zeka ile, paket iskeleti, SDF düzeltmeleri (Sensors plugin'i, kamera konumu) ve hata ayıklama için yararlanıldı.
