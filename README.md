# up_week2 — Kırmızı Hedef Tespiti (ROS 2 Humble + Gazebo Fortress + OpenCV)

Under Pressure AUV Takımı yazılım birim alım süreci, **2. hafta** ödevi.
Birinci haftanın (`up_week1`) sahne ve düğümleri korunur; bunun üzerine
hareketli kırmızı hedef, klasik görüntü işleme tabanlı tespit, ROS çıktıları,
iki aydınlatma kipi ve başarım ölçüm aracı eklenmiştir.

**Aday:** Idil Sincer

---

## 1. Ortam, sürümler ve bağımlılıklar

| Bileşen | Sürüm / not |
|---|---|
| İşletim sistemi | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Simülasyon | Gazebo Fortress (`ign gazebo`) |
| Dil | Python 3.10, `ament_python` |
| Görüntü işleme | OpenCV (`python3-opencv`) + NumPy |
| Köprü | `ros_ign_bridge` / `ros_gz_bridge` (ikisi de otomatik algılanır) |

```bash
sudo apt update
sudo apt install -y \
  ros-humble-vision-msgs \
  ros-humble-ros-ign-bridge ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-interfaces \
  python3-opencv python3-numpy python3-yaml \
  python3-colcon-common-extensions
```

> Yeni Humble kurulumlarında paketler `ros_gz_*` adıyla gelebilir. Launch,
> kodu ve `target_mover` her iki adlandırmayı da destekler; README'de
> `ros_ign_*` yazılmıştır.

`up_week1` bu paketin çalışma zamanı bağımlılığıdır (`package.xml` içinde
tanımlıdır). Derlemeden önce `~/ros2_ws/src` altında bulunmalıdır.

### up_week1 launch dosyası hakkında (önemli)

Ödev, birinci haftanın launch dosyasının **kopyalanmadan dahil edilmesini**
istiyor. Week-1 launch dosyası dünyayı sabit olarak seçtiği için, week-2
dünyasını yükleyebilmek amacıyla geriye dönük uyumlu tek bir opsiyonel
argüman eklendi:

```text
world:=<gazebo dünya dosyası>     (varsayılan: up_week1/worlds/test_world.sdf)
```

Yama dosyası: [`patches/up_week1_world_arg.patch`](patches/up_week1_world_arg.patch)

```bash
cd ~/ros2_ws/src/up_week1
git apply /path/to/up_week2/patches/up_week1_world_arg.patch
```

`week2.launch.py` yamayı otomatik algılar. Yamalanmamış bir `up_week1`
bulursa hafta-1 launch dosyasını kopyalamadan, hafta-1 **executable**'larını
doğrudan başlatır ve week-2 dünyasını yükler. Yani iki durumda da tek launch
dosyası yeterlidir.

---

## 2. Derleme ve çalıştırma

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Tek komutla her şey (dünya + kamera köprüsü + hafta-1 düğümleri +
`target_mover` + `red_target_detector`):

```bash
ros2 launch up_week2 week2.launch.py
```

### Launch argümanları

| Argüman | Değerler | Varsayılan | Açıklama |
|---|---|---|---|
| `lighting` | `normal`, `dim` | `normal` | Normal veya loş dünya. `dim` seçilirse `config/detector_dim.yaml` otomatik yüklenir. |
| `use_mover` | `true`, `false` | `true` | Hedefi yol üzerinde hareket ettir / başlangıç noktasında sabit tut. |
| `params_file` | YAML yolu | boş | Boşsa aydınlatmaya göre `config/detector.yaml` veya `config/detector_dim.yaml` seçilir. |

Örnekler:

```bash
# Loş aydınlatma, loş parametreleriyle
ros2 launch up_week2 week2.launch.py lighting:=dim

# Hedef hareket etmeden (başlangıç konumunda)
ros2 launch up_week2 week2.launch.py use_mover:=false

# Elle seçilmiş parametre dosyası
ros2 launch up_week2 week2.launch.py \
  lighting:=dim params_file:=$HOME/ros2_ws/src/up_week2/config/detector_dim.yaml
```

Görüntüleme araçları ayrı başlatılır:

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 run rqt_image_view rqt_image_view /vision/debug_image
ros2 topic echo /vision/detections
ros2 topic echo /vision/target_offset
ros2 topic echo /vision/target_detected
```

Sanal makinede kamera render'ı çalışmazsa:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch up_week2 week2.launch.py
```

---

## 3. Paket yapısı

```text
up_week2/
├── launch/week2.launch.py          # tek başlatma dosyası
├── worlds/
│   ├── test_world_w2.sdf           # normal aydınlatma (up_test_world)
│   └── test_world_w2_dim.sdf       # loş aydınlatma (aynı sahne, ~yarı parlaklık)
├── config/
│   ├── target_path.yaml            # hedef yolu + zaman damgaları
│   ├── detector.yaml               # normal aydınlatma parametreleri
│   └── detector_dim.yaml           # loş aydınlatma parametreleri
├── up_week2/
│   ├── heartbeat_publisher.py      # hafta-1 yedeği (launch up_week1'i kullanır)
│   ├── heartbeat_subscriber.py     # hafta-1 yedeği
│   ├── camera_monitor.py           # hafta-1 yedeği
│   ├── target_mover.py             # hedefi yol üzerinde hareket ettirir
│   ├── red_target_detector.py      # klasik CV tespit düğümü
│   ├── evaluate_detection.py       # başarım ölçüm aracı
│   └── scene_geometry.py           # kamera projeksiyonu / görünürlük yardımcıları
├── patches/up_week1_world_arg.patch
├── package.xml
├── setup.py
└── README.md
```

---

## 4. Simülasyon dünyası

Dünya adı: **`up_test_world`**.

| Nesne | Konum / açıklama |
|---|---|
| `ground_plane` | 12 × 12 m zemin (ister: en az 8 × 8 m) |
| `occluder_box` | (0, −0.6, 0.8), 1.2 × 0.3 × 1.6 m; kamera ile hedef yolu arasında |
| `red_target` | Kırmızı kutu (0.4 × 0.4 × 1.0 m); başlangıç (−1.2, −1.5, 0.5) |
| `decoy_green` | Belirgin farklı renk (yeşil), (2.0, 1.4, 0.5) |
| `decoy_burgundy` | Kırmızıya yakın renk (bordo), (−1.8, −0.3, 0.5) |
| `Camera` | Sabit, (0, −5.0, 1.8), yaw 90° (+y), pitch 20° aşağı; 640×480, HFOV 60°, 10 Hz |

- Kameranın görüş alanında aynı anda yalnızca bir gerçek hedef vardır.
- Hedefin ve çeldiricilerin görsel + çarpışma geometrileri vardır, isimleri benzersizdir.
- `red_target` çalışma zamanında `/world/up_test_world/set_pose` ile taşınır.
  Gazebo `set_pose` servisi statik modelleri taşımadığı için hedef dinamiktir;
  düz tabanlı kutu geometrisi, 30 Hz konum güncellemeleri arasında devrilmeden
  dengede kalmasını sağlar.
- İki dünya dosyası yalnızca aydınlatma değerlerinde farklıdır:
  normalde sahne ambient 0.4 / ışık şiddeti 1.0, loşta 0.2 / 0.5.

### Hedef yolu (`config/target_path.yaml`)

Yol noktaları `t` (saniye) ve `(x, y, z)` olarak verilir; iki nokta arasında
hedef doğrusal olarak taşınır. Döngü periyodu **38 s**'dir ve her çalıştırmada
birebir aynıdır.

| Zaman (s) | Hareket | İster karşılığı |
|---|---|---|
| 0–6 | Başlangıçta tam görünür, sabit | En az 5 s sabit durma |
| 6–12 | Görüş alanı içinde 1.2 m + 1.2 m yer değiştirme | En az 1 m hareket |
| 12–15 | Engelin yanından arkasına geçer, hâlâ görünür | — |
| 15–19 | Yandan engelin arkasına girer: kısmi → tam kapanma | Kısmi/tam kapanma |
| 19–23 | Tamamen engel arkasında | Kapanma |
| 23–26 | Engel arkasından çıkar, görüş alanından ayrılır | Yeniden görünme + çıkış |
| 26–30 | Görüş alanı dışında | En az 3 s görüş alanı dışı |
| 30–35 | Engel etrafından dolaşarak görüş alanına girer | Yeniden giriş |
| 35–38 | Başlangıç konumunda bekler, döngü kesintisiz başlar | Tekrarlanabilirlik |

`target_mover`, yolun yanında ölçüm aracı için şu ek topic'leri yayınlar:

- `/target/ground_truth` — `geometry_msgs/msg/PoseStamped`, hedefin bilinen
  (komut edilen) dünya konumu,
- `/target/visibility` — `std_msgs/msg/String`; `full`, `partial`, `hidden`,
  `out_of_view`. Kamera pozu ve engel kutusu kullanılarak geometrik olarak
  hesaplanır.

Bu iki topic ödev çıktısı değil, **değerlendirme referansıdır**.

---

## 5. Düğümler, topic'ler ve mesaj tipleri

| Düğüm | Görev | Topic | Mesaj tipi | Yön |
|---|---|---|---|---|
| `heartbeat_publisher` (up_week1) | Saniyede bir heartbeat | `/up/heartbeat` | `std_msgs/msg/String` | yayın |
| `heartbeat_subscriber` (up_week1) | Heartbeat dinler | `/up/heartbeat` | `std_msgs/msg/String` | abone |
| `camera_monitor` (up_week1) | Kamera akışını izler | `/camera/image_raw` | `sensor_msgs/msg/Image` | abone |
| `parameter_bridge` | Kamera, saat, camera_info ve set_pose köprüsü | `/camera/image_raw`, `/camera/camera_info`, `/clock`, `/world/up_test_world/set_pose` | Image, CameraInfo, Clock, `ros_ign_interfaces/srv/SetEntityPose` | köprü |
| `target_mover` | Hedefi yol üzerinde taşır + ground truth | `/world/up_test_world/set_pose`, `/target/ground_truth`, `/target/visibility` | SetEntityPose (istemci), PoseStamped, String | çağrı/yayın |
| `red_target_detector` | Kırmızı hedefi tespit eder | `/camera/image_raw` | `sensor_msgs/msg/Image` | abone |
| | | `/vision/detections` | `vision_msgs/msg/Detection2DArray` | yayın |
| | | `/vision/target_detected` | `std_msgs/msg/Bool` | yayın |
| | | `/vision/target_offset` | `geometry_msgs/msg/PointStamped` | yayın |
| | | `/vision/debug_image` | `sensor_msgs/msg/Image` | yayın |
| `evaluate_detection` | Başarım ölçer | `/vision/detections`, `/target/ground_truth`, `/target/visibility` | — | abone |

### Çıktı sözleşmeleri

- **`/vision/detections`**: Her aday için sınırlayıcı kutunun merkezi
  (`bbox.center.position.x/y`, piksel) ve boyutu (`bbox.size_x/size_y`, piksel).
  Adaylar **alana göre azalan** sırada. Aday yoksa boş dizi yayınlanır.
- **`/vision/target_detected`**: En büyük geçerli adayın onaylanmış varlığı
  (`true/false`). Onay/kayıp histerezisi uygulanır.
- **`/vision/target_offset`**: `x = (cx - W/2)/(W/2)` (sağa pozitif),
  `y = (cy - H/2)/(H/2)` (aşağı pozitif), `z = alan / (W*H)`.
  Yalnızca onaylanmış tespit varken yayınlanır.
- **`/vision/debug_image`**: Tüm adayların kutuları + en büyük adayın merkez
  işareti. `bgr8`. `rqt_image_view` ile gösterilebilir.
- Tüm çıktıların `header.stamp` ve `header.frame_id` alanları, işlenen
  görüntünün başlığından **birebir kopyalanır**.

### QoS

| Topic | QoS |
|---|---|
| `/camera/image_raw` aboneliği | **BEST_EFFORT**, KEEP_LAST, depth **1**, VOLATILE (`qos_profile_sensor_data` benzeri) |
| `/vision/debug_image` | BEST_EFFORT, KEEP_LAST, depth 1 |
| `/vision/detections`, `/vision/target_detected`, `/vision/target_offset` | RELIABLE, KEEP_LAST, depth 10 |

Kamera aboneliğinde depth 1 + best effort seçilmesi, işleme yetişmediğinde
eski karelerin kuyrukta birikmesini ve gecikme oluşmasını engeller.

---

## 6. Tespit algoritması

1. **Kod çözme:** `rgb8`, `bgr8` (ayrıca `rgba8`, `bgra8`) elle NumPy ile
   işlenir; `step` (satır dolgusu) dikkate alınır. Bozuk/boş görüntü
   yakalanır, düğüm çökmez.
2. **HSV eşikleme:** Kırmızı hue çemberde iki uçta olduğu için iki aralık:
   `H ∈ [0, 8]` ve `H ∈ [175, 180]`; `S ≥ min_saturation`, `V ≥ min_value`.
   Varsayılan aralıklar bordo çeldiricinin hue'sunu (~172) dışarıda bırakır.
3. **Morfoloji:** 3×3 açma (tek piksel gürültüsü) + kapama (delikler).
4. **Kontur/alan filtresi:** Dış konturlar; `min_area ≤ alan ≤
   max_area_ratio·W·H`. Adaylar alana göre azalan sıralanır.
5. **Histerezis:** `confirm_frames` ardışık isabetten sonra `detected=true`,
   `lost_frames` ardışık ıskadan sonra `false`. Tek karelik gürültü durumu
   değiştirmez.
6. **Akış koruması:** 1 saniyeden uzun görüntü kesintisinde durum `false`
   yapılır, boş dizi yayınlanır; akış dönünce kendiliğinden devam edilir.
7. **İstatistik:** Saniyede bir işlenen kare sayısı, tespit durumu, merkez
   (piksel), alan ve ortalama işleme süresi (ms) terminale yazılır.

Derin öğrenme modeli, hazır renk takip/tanıma düğümü veya `cv_bridge` içindeki
hazır tespit hattı kullanılmamıştır.

---

## 7. Parametreler

`config/detector.yaml` (normal) ve `config/detector_dim.yaml` (loş) aynı
düğüm tarafından yüklenir. Çalışırken `ros2 param set` ile değiştirilebilir;
geçersiz değerler `add_on_set_parameters_callback` tarafından **reddedilir**.

| Parametre | Anlam | Normal | Loş | Doğrulama |
|---|---|---|---|---|
| `hsv_lower_h` | 1. kırmızı hue alt sınırı | 0 | 0 | 0–180, ≤ upper |
| `hsv_upper_h` | 1. kırmızı hue üst sınırı | 8 | 8 | 0–180 |
| `hsv_lower_h2` | 2. kırmızı hue alt sınırı | 175 | 175 | 0–180, ≤ upper2 |
| `hsv_upper_h2` | 2. kırmızı hue üst sınırı | 180 | 180 | 0–180 |
| `min_saturation` | En düşük doygunluk | 120 | 100 | 0–255 |
| `min_value` | En düşük parlaklık | 70 | 35 | 0–255 |
| `min_area` | En küçük kabul edilen alan (px²) | 250.0 | 200.0 | ≥ 0 |
| `max_area_ratio` | En büyük alan / görüntü alanı | 0.5 | 0.5 | (0, 1] |
| `confirm_frames` | Onay için ardışık kare | 3 | 3 | ≥ 1 |
| `lost_frames` | Kayıp için ardışık kare | 5 | 5 | ≥ 1 |
| `debug_image` | Debug görüntüsü yayınla | true | true | bool |
| `stream_timeout` | Akış kesinti eşiği (s) | 1.0 | 1.0 | > 0 |
| `stats_period` | Terminal istatistik periyodu (s) | 1.0 | 1.0 | > 0 |

Örnek canlı değişiklik:

```bash
# Daha küçük lekeleri de kabul et
ros2 param set /red_target_detector min_area 100.0

# Geçersiz değer reddedilir (terminalde "Rejected parameter update" görülür)
ros2 param set /red_target_detector min_area -5.0
```

### Eşiklerin seçilme gerekçesi

- Gerçek hedef saf kırmızı (`1, 0, 0`) malzeme ile modellendiğinden hue'su
  0'a çok yakındır. Bordo çeldirici (`0.45, 0, 0.12`) hue ≈ 344° (OpenCV'de
  172) verir; `175–180` penceresi dışında kalır.
- Tam görünür hedefin izdüşüm alanı yaklaşık 3000–7000 px² olduğundan
  `min_area=250` gürültüyü eler, hedefi kaçırmaz.
- `confirm_frames=3` / `lost_frames=5` (10 Hz'de 0.3 s / 0.5 s) tek karelik
  paraziti bastırırken kapanma başlangıcına hızlı tepki verir.
- Loş dünyada hedefin `V` değeri düştüğü için yalnızca `min_value` ve
  `min_saturation` düşürülür; kod değişmez.

---

## 8. Başarım ölçümü

`evaluate_detection`, referans olarak **sahnedeki bilinen hedef konumunu**
(`/target/ground_truth`) kullanır. Konum, kamera modeliyle piksele izdüşürülür
(`scene_geometry.project_point`); görünürlük etiketi `/target/visibility`'den
alınır.

- **Tespit oranı:** Yalnızca `visibility=full` karelerde, en az bir tespit
  bulunan karelerin oranı.
- **Yanlış tespit oranı:** `hidden` + `out_of_view` karelerde tespit
  yayınlanan karelerin oranı. Çeldiriciler bu fazlarda görünür olduğu için
  bu oran aynı zamanda çeldirici reddini gösterir.
- **Ortalama merkez hatası:** `full` karelerde tespit merkezi ile izdüşürülen
  referans merkez arasındaki Öklid mesafesi (piksel).

Hedefler: tespit oranı ≥ %90, yanlış tespit ≤ %5, merkez hatası ≤ 10 px.
Çalışma en az 100 kare ve yolun tamamını (38 s) kapsamalıdır; iki aydınlatma
için ayrı ayrı çalıştırılmalıdır.

```bash
# Terminal 1
ros2 launch up_week2 week2.launch.py lighting:=normal

# Terminal 2 (Gazebo ve düğümler hazır olduktan sonra)
ros2 run up_week2 evaluate_detection --ros-args \
  -p duration:=45.0 -p lighting_label:=normal \
  -p output_csv:=$HOME/eval_normal.csv

# Loş aydınlatma için ayrı çalıştırma
ros2 launch up_week2 week2.launch.py lighting:=dim
ros2 run up_week2 evaluate_detection --ros-args \
  -p duration:=45.0 -p lighting_label:=dim \
  -p output_csv:=$HOME/eval_dim.csv
```

Araç bitince terminale özet basar, `PASS/FAIL` yazar ve kare bazlı CSV üretir.

### Ölçüm sonuçları (VM'de doldurulacak)

| Aydınlatma | Kare | Tespit oranı | Yanlış tespit | Ort. merkez hatası | Sonuç |
|---|---|---|---|---|---|
| normal |  |  |  |  |  |
| dim |  |  |  |  |  |

> Bu tablo, kodu yazdığım makinede ROS/Gazebo olmadığı için boş bırakılmıştır.
> Kullanıcı VM'de iki komutu çalıştırıp çıktıları buraya ve rapora işlemelidir.

---

## 9. Doğrulama senaryoları (demo videosu için)

1. `ros2 launch up_week2 week2.launch.py` — tek launch ile sahne + tüm düğümler.
2. `rqt_image_view /vision/debug_image` — hedefin kutu ve merkezle izlenmesi.
3. `ros2 topic echo /vision/target_offset` — hedef sağa/sola/yukarı/aşağı
   giderken x/y'nin işaret değiştirmesi; `target_detected`'ın false→true.
4. Kapanma fazında (yolun 15–23 s aralığı) debug görüntüsünde hedefin
   kaybolması, `target_detected=false`, `detections` boş.
5. Görüş alanı dışı (26–30 s) ve yeniden giriş.
6. `ros2 param set /red_target_detector min_area 5000.0` — tespitin kaybolması;
   geri `250.0` — düzelmesi. Geçersiz `-5.0` değerinin reddi.
7. Kamera akışını kesme: ayrı terminalde köprü sürecini durdurup yaklaşık
   1 s sonra `/vision/target_detected`'ın false olduğunu, ardından köprüyü
   tekrar başlatınca tespitin kendiliğinden döndüğünü gösterin:

   ```bash
   pkill -f "parameter_bridge.*camera/image_raw"
   # 1-2 s bekleyin: ros2 topic echo /vision/target_detected  -> false
   ros2 run ros_ign_bridge parameter_bridge \
     '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image'
   ```
8. `lighting:=dim` ile aynı tespitin çalışması.
9. Hafta-1 heartbeat ve camera_monitor loglarının hâlâ akması.
10. Çeldirici testi: hedef engel arkasındayken (yalnızca yeşil + bordo
    görünürken) `/vision/detections` boş kalmalı.

---

## 10. Yeniden üretilebilirlik

- Tüm dosya yolları paketin kurulu `share` dizininden veya `$HOME`'dan
  türetilir; kullanıcıya özgü sabit yol yoktur.
- Dünya, parametreler ve hedef yolu kaynak dosyalarda tanımlıdır.
- Simülasyon kapatılıp yeniden açıldığında hedef aynı başlangıç noktasından
  aynı 38 s'lik yolu izler.
- Derleme çıktıları (`build/`, `install/`, `log/`) depoya eklenmemelidir
  (`.gitignore`).

---

## 11. Bilinen sınırlamalar

- `target_mover` hedefi `set_pose` ile ışınlar (teleport). 30 Hz'de küçük
  adımlarla yapıldığı için görüntüde akıcı görünür; fiziksel bir hız
  denetleyicisi değildir.
- `ros_ign_bridge` servis köprüsü çalışmazsa `target_mover` otomatik olarak
  `ign service` CLI geri dönüşüne geçer (5 Hz); bu durumda hareket biraz daha
  kesikli olabilir.
- Ölçüm aracının referansı, hedefin komut edilen konumudur. Servis
  gecikmesi nedeniyle hızlı hareket fazlarında birkaç piksel gecikme payı
  olabilir; değerlendirme en yakın zaman damgalı ground truth'u seçer.
- Loş aydınlatma eşikleri gerçek render çıktısına göre ince ayar
  gerektirebilir; kod değişmeden yalnızca `detector_dim.yaml` düzenlenir.

---

## 12. Dış kaynaklar ve yapay zekâ desteği

- ROS 2 Humble ve Gazebo Fortress resmî dokümantasyonu.
- `vision_msgs/Detection2DArray` mesaj tanımı.
- OpenCV `inRange`, `findContours`, `morphologyEx` dokümantasyonu.
- `ros_ign_bridge` servis köprüsü kullanımı (SetEntityPose) için resmî
  `ros_gz_bridge` README'si ve Gazebo `set_pose` servis örnekleri.
- **Yapay zekâ desteği:** Kod iskeletinin ve README/rapor taslağının hazırlanmasında yapay zekâ asistanından yararlanılmıştır. 
