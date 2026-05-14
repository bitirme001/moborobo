# How To Run

Bu dosya, hazır node/waypoint yapısı varken MQTT alarm alıp `move_base` ile optimal rota çalıştırmak için sıralı çalışma adımlarını içerir.

## 1. Ön koşullar

Workspace:

```bash
cd /Users/nisa/Desktop/moborobo
catkin_make
source devel/setup.bash
```

Gerekli runtime paketleri:

```bash
sudo apt update
sudo apt install -y python3-paho-mqtt mosquitto-clients mosquitto
```

Not:

- `python3-paho-mqtt` varsa node doğrudan broker’a bağlanır.
- `python3-paho-mqtt` yoksa `mosquitto_sub` fallback olarak kullanılabilir.
- Alarm publish testi için `mosquitto_pub` gerekir.

## 1.1. IP adresini nerede yazacaksın

Bu sistemde MQTT broker Ubuntu / robot PC üzerinde çalışmalı.

Yani:

- ESP32 `MQTT_SERVER` değeri = Ubuntu PC IP adresi
- Mock publisher PC broker IP değeri = Ubuntu PC IP adresi
- ROS launch içindeki `mqtt_host` değeri = Ubuntu PC IP adresi

Ubuntu IP öğrenmek için:

```bash
hostname -I
```

Örnek Ubuntu IP:

```text
192.168.1.100
```

ESP32 tarafında değiştirilecek satır:

```cpp
const char* MQTT_SERVER = "192.168.1.100";
```

ROS tarafında değiştirilecek dosya:

`/Users/nisa/Desktop/moborobo/src/smart_waste_nav/launch/alarm_route_executor.launch`

Bu launch dosyasında en üstteki satırlar:

```xml
<arg name="mqtt_host" default="192.168.1.100" />
<arg name="mqtt_port" default="1883" />
```

Aynı Wi-Fi üzerinde:

- ESP32 bu IP’ye publish eder
- Mock publisher PC bu IP’ye publish eder
- robot bu IP’deki broker’dan alarm dinler
- robot yine aynı IP’deki broker’a `bin/cleared` publish eder

Önerilen topoloji:

```text
ESP32 (node 1) --------\
                        \
Mock publisher PC -------> Robot PC (Mosquitto broker + ROS)
                        /
Robot clear publisher --/
```

## 2. Node dosyasını kontrol et

Hazır node’lar burada:

`/Users/nisa/Desktop/moborobo/src/smart_waste_nav/config/navigation_nodes.yaml`

Kontrol edilmesi gereken alanlar:

- `id`
- `name`
- `mqtt_id`
- `clear_id`
- `x`
- `y`
- `yaw`
- `neighbors`

Alarm eşleme şu sırayla çalışır:

1. Önce gelen alarmdaki `name`, node `name` veya `aliases` ile eşleşir.
2. İsim eşleşmezse `lat/lng` ile en yakın node aranır.

Şu an `lat/lng` alanları boşsa, test için alarm JSON içindeki `name` alanı node adıyla uyuşmalıdır.

Örnek geçerli isimler:

- `Trash Bin 1`
- `Trash Bin 2`
- `Trash Bin 3`
- `Trash Bin 4`

## 3. Robot ve navigasyon altyapısını başlat

Önce ROS master:

```bash
roscore
```

Robot motor + odom:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
sudo chmod 666 /dev/ttyACM0
roslaunch moborobot motor_only.launch
```

LiDAR:

```bash
sudo ifconfig enp0s31f6 192.168.1.102 netmask 255.255.255.0 up

cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch rslidar_pointcloud rs_lidar_16.launch
```

## 3.1. Gerekli static TF transformlarını yayınla

Bu navigation akışında `base_link` ana gövde frame'i olarak kullanılıyor.
LiDAR için `rslidar -> base_link` transformu gerekli.
İsterseniz `base_link -> base_footprint` transformunu da ayrıca yayınlayabilirsiniz.

`rslidar -> base_link`:

```bash
rosrun tf static_transform_publisher 0 0 0.4 0 0 0 1 base_link rslidar 100
```

`base_link -> base_footprint`:

```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link base_footprint 100
```

Not:

- Eğer kullandığınız başka bir launch bu TF'leri zaten publish ediyorsa bunları tekrar çalıştırmayın.
- `rslidar -> base_link` için `0 0 0.4 0 0 0 1` değeri repodaki eski launch örneğinden alınmıştır; gerçek montaj offset'inize göre güncelleyin.
- Mevcut `smart_waste_nav` tarafı `base_link` ile çalışır; `base_footprint` zorunlu değil ama bazı araçlar için faydalı olabilir.

PointCloud2 -> LaserScan:

```bash
rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node \
cloud_in:=/rslidar_points \
scan:=/scan \
_target_frame:=base_link \
_min_height:=-1.0 \
_max_height:=1.0 \
_range_min:=0.3 \
_range_max:=30.0 \
_transform_tolerance:=0.5
```

Map server:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav map.launch
```

AMCL:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav amcl.launch
```

`move_base`:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav move_base.launch
```

## 4. Hazır node’ları sırayla test et

Node koordinatları doğru mu görmek için önce klasik waypoint testi çalıştır:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav waypoint_executor.launch
```

Bu aşamada robot sırasıyla `navigation_nodes.yaml` içindeki aktif node’lara gider.

## 5. Alarm algoritmasını demo modda test et

Gerçek MQTT olmadan rota algoritmasını hızlı test:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav alarm_route_demo.launch
```

Bu launch varsayılan olarak `bin_2` ve `bin_4` alarmı varmış gibi davranır.

## 6. MQTT broker başlat

Eğer broker aynı makinede çalışacaksa:

```bash
mosquitto -v
```

Eğer servis olarak kuruluysa:

```bash
sudo systemctl start mosquitto
sudo systemctl status mosquitto
```

Broker başka makinedeyse sadece IP/port bilgisini launch içinde override etmen yeterli.

## 7. MQTT alarm dinleyen executor’ı başlat

Varsayılan broker:

- host: `mqtt_host` arg içindeki IP
- port: `1883`
- topic: `bin/status`
- clear topic: `bin/cleared`

Çalıştır:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav alarm_route_executor.launch
```

Eğer broker başka makinedeyse:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav alarm_route_executor.launch \
mqtt_host:=192.168.1.50 \
mqtt_port:=1883 \
mqtt_topic:=bin/status
```

Bu node:

1. MQTT alarmını dinler
2. Alarmı node’a eşler
3. Bekleyen alarmlar arasında optimal sıra çıkarır
4. `move_base` hedeflerini sırayla gönderir
5. Alarm node’una ulaştığında `bin/cleared` topic’ine `{"id":X,"emptied":true}` publish eder
6. Yeni alarm gelirse rotayı yeniden planlar

Not:

- Bu launch içinde `alarm_filter_enabled=true`.
- Yani `bin/status` üstündeki `alarm=false` paketleri rota üretmez.
- Varsayılan olarak `use_explicit_neighbors=false` ve `execute_intermediate_nodes=false`.
- Yani alarm modunda robot aradaki bin node'larına uğramaz; `move_base` doğrudan alarm hedeflerine gider.

## 8. Mock data publisher bilgisayarını başlat

Mock publisher ayrı bir bilgisayarda çalışmalı ve doğrudan robot PC üzerindeki broker’a bağlanmalı.

Ön koşul:

```bash
pip3 install paho-mqtt
```

Dosya:

`/Users/nisa/Desktop/moborobo/mock_bin_status_publisher.py`

Node `2-7` için `fillPercent` değerlerini bu dosyanın içindeki `MOCK_BINS` listesinden değiştirebilirsiniz.
Değişiklikten sonra script'i yeniden başlatın.

Çalıştır:

```bash
cd /Users/nisa/Desktop/moborobo
python3 mock_bin_status_publisher.py \
  --host 192.168.1.100
```

Bu script:

1. `bin/status` topic’ine node `2-7` durumlarını publish eder
2. `bin/cleared` topic’ini dinler
3. Robot bir node’u temizlediğinde o node’un mock alarmını kapatır

Not:

- Bir node alarm verdikten sonra sadece `fillPercent` düşürmek alarmı hemen kapatmaz.
- Bu davranış gerçek sensör tarafıyla aynı olsun diye latched tutulur.
- Elle kapatmak istersen:

```bash
mosquitto_pub -h 192.168.1.100 -p 1883 -t bin/cleared -m '{"id":4,"emptied":true}'
```

## 9. MQTT üzerinden test alarmı gönder

Tek alarm:

```bash
mosquitto_pub -h localhost -p 1883 -t bin/status -m '{"id":2,"nodeId":"bin_2","name":"Trash Bin 2","fillPercent":90,"isFull":true,"alarm":true}'
```

Birden fazla alarm:

```bash
mosquitto_pub -h localhost -p 1883 -t bin/status -m '{"alarms":[{"id":4,"nodeId":"bin_4","name":"Trash Bin 4","fillPercent":95,"isFull":true,"alarm":true},{"id":2,"nodeId":"bin_2","name":"Trash Bin 2","fillPercent":86,"isFull":true,"alarm":true}]}'
```

Not:

- `id`, `nodeId`, `name` alanlarından biri eşleştiğinde node bulunur.
- `lat/lng` ile eşleme kullanacaksan `navigation_nodes.yaml` içine gerçek GPS değerlerini gir.

## 10. Yeni map için yol üstünden node kaydet

Robotu manuel sürerek yol üzerinde node toplamak için:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav node_path_recorder.launch
```

Çıkış dosyası:

`/Users/nisa/Desktop/moborobo/src/smart_waste_nav/config/recorded_nodes.yaml`

Bu dosyadaki node’ları temizleyip son halini `navigation_nodes.yaml` içine taşıyabilirsin.

## 11. Önerilen gerçek kullanım sırası

1. `roscore`
2. motor / odom
3. lidar
4. pointcloud_to_laserscan
5. `smart_waste_nav map.launch`
6. `smart_waste_nav amcl.launch`
7. `smart_waste_nav move_base.launch`
8. `smart_waste_nav waypoint_executor.launch` ile node doğrulama
9. broker başlat
10. `smart_waste_nav alarm_route_executor.launch`
11. ESP32 node `1` publish etmeye başlasın
12. mock publisher PC node `2-7` publish etmeye başlasın

## 12. Beklenen davranış

Alarm geldiğinde sistem:

1. Alarmı JSON olarak parse eder
2. Hangi node’a ait olduğunu bulur
3. Aynı anda birden fazla alarm varsa en uygun ziyaret sırasını hesaplar
4. Node graph üstünde geçilecek yolu çıkarır
5. Robotu `move_base` ile bu hedeflere gönderir
6. Hedef alarm node’una ulaşınca `bin/cleared` publish eder

## 13. Sorun olursa ilk kontrol listesi

- `rostopic echo /amcl_pose`
- `rostopic echo /move_base/status`
- `rostopic echo /scan`
- `rostopic echo /map`
- `navigation_nodes.yaml` içindeki `neighbors` doğru mu
- Alarm JSON içindeki `id`, `nodeId` veya `name` gerçekten node ile uyuşuyor mu
- Broker host/port/topic doğru mu
- `python3-paho-mqtt` veya `mosquitto_sub`/`mosquitto_pub` kurulu mu
