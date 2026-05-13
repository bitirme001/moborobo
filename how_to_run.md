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

## 2. Node dosyasını kontrol et

Hazır node’lar burada:

`/Users/nisa/Desktop/moborobo/src/smart_waste_nav/config/navigation_nodes.yaml`

Kontrol edilmesi gereken alanlar:

- `id`
- `name`
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
roslaunch moborobot motor_only.launch
```

LiDAR:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch rslidar_pointcloud rs_lidar_16.launch
```

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

- host: `localhost`
- port: `1883`
- topic: `waste/alarm`

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
mqtt_topic:=waste/alarm
```

Bu node:

1. MQTT alarmını dinler
2. Alarmı node’a eşler
3. Bekleyen alarmlar arasında optimal sıra çıkarır
4. `move_base` hedeflerini sırayla gönderir
5. Yeni alarm gelirse rotayı yeniden planlar

## 8. MQTT üzerinden test alarmı gönder

Tek alarm:

```bash
mosquitto_pub -h localhost -p 1883 -t waste/alarm -m '{"name":"Trash Bin 2","lat":39.8721,"lng":32.7352,"weightKg":12.5,"fillPercent":70,"isFull":false}'
```

Birden fazla alarm:

```bash
mosquitto_pub -h localhost -p 1883 -t waste/alarm -m '{"alarms":[{"name":"Trash Bin 4","lat":39.8721,"lng":32.7352,"weightKg":12.5,"fillPercent":95,"isFull":true},{"name":"Trash Bin 2","lat":39.8721,"lng":32.7352,"weightKg":10.0,"fillPercent":70,"isFull":false}]}'
```

Not:

- Şu an isim eşleşmesi en güvenli test yöntemi.
- `lat/lng` ile eşleme kullanacaksan `navigation_nodes.yaml` içine gerçek GPS değerlerini gir.

## 9. Yeni map için yol üstünden node kaydet

Robotu manuel sürerek yol üzerinde node toplamak için:

```bash
cd /Users/nisa/Desktop/moborobo
source devel/setup.bash
roslaunch smart_waste_nav node_path_recorder.launch
```

Çıkış dosyası:

`/Users/nisa/Desktop/moborobo/src/smart_waste_nav/config/recorded_nodes.yaml`

Bu dosyadaki node’ları temizleyip son halini `navigation_nodes.yaml` içine taşıyabilirsin.

## 10. Önerilen gerçek kullanım sırası

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
11. `mosquitto_pub` ile alarm gönder

## 11. Beklenen davranış

Alarm geldiğinde sistem:

1. Alarmı JSON olarak parse eder
2. Hangi node’a ait olduğunu bulur
3. Aynı anda birden fazla alarm varsa en uygun ziyaret sırasını hesaplar
4. Node graph üstünde geçilecek yolu çıkarır
5. Robotu `move_base` ile bu hedeflere gönderir

## 12. Sorun olursa ilk kontrol listesi

- `rostopic echo /amcl_pose`
- `rostopic echo /move_base/status`
- `rostopic echo /scan`
- `rostopic echo /map`
- `navigation_nodes.yaml` içindeki `neighbors` doğru mu
- Alarm JSON içindeki `name`, node adıyla gerçekten uyuşuyor mu
- Broker host/port/topic doğru mu
- `python3-paho-mqtt` veya `mosquitto_sub` kurulu mu
